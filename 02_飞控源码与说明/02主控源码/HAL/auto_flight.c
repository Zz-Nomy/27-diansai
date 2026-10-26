#include "auto_flight.h"
#include "flow.h"
#include "control.h"
#include "remote.h"
#include "light_task.h"
#include "myMath.h"
#include <math.h>
#include <string.h>

volatile AutoFlightStatus auto_flight;
static volatile uint8_t rc_seen, button_seen, button_candidate;
static uint8_t button_level;
static uint8_t last_aux1, landing_enabled, alarm_enabled, estimator_ready;
static uint8_t airborne;
static volatile uint32_t last_rc_tick, button_tick;
static uint32_t ground_tick, state_tick;
static uint32_t estimator_tick;
static float ground_height, ground_reference, old_height, old_velocity;
static float ramp_seconds, flow_lost_seconds, landing_gesture_seconds;
static float landing_arm_seconds;
static float touchdown_seconds, low_seconds, high_seconds;
static float history[3];
static uint8_t history_index;

static float absf(float value) { return value < 0.0f ? -value : value; }
static float median3(float a, float b, float c)
{
    float t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    return a > b ? a : b;
}
static uint8_t range_fresh(void)
{
    return flow_height_valid &&
        (uint32_t)(SysTick_count - flow_last_valid_tick) <= 4U;
}
static void reset_height_pid(void)
{
    pidHeightHigh.integ = pidHeightHigh.prevError = pidHeightHigh.out = 0.0f;
    pidHeightRate.integ = pidHeightRate.prevError = pidHeightRate.out = 0.0f;
    pidHeightRate.desired = pidHeightRate.measured = 0.0f;
    old_velocity = 0.0f;
}
static void stop_auto(void)
{
    ALL_flag.take_off = ALL_flag.take_down = 0;
    ALL_flag.height_lock = ALL_flag.flow_control = 0;
    auto_flight.state = AUTO_IDLE;
    auto_flight.throttle = 0.0f;
    auto_flight.stable_seconds = 0.0f;
    auto_flight.takeover_pending = 0;
    alarm_enabled = landing_enabled = airborne = 0;
    reset_height_pid();
}
static void land(AutoFlightError error)
{
    if (auto_flight.state == AUTO_LANDING) return;
    auto_flight.state = AUTO_LANDING;
    auto_flight.error = error;
    auto_flight.alarm = error == AUTO_OK ? AUTO_ALARM_NONE : AUTO_ALARM_SENSOR;
    ALL_flag.take_down = 1;
    state_tick = SysTick_count;
    touchdown_seconds = 0.0f;
}

void AutoFlight_Init(void)
{
    memset((void *)&auto_flight, 0, sizeof(auto_flight));
    rc_seen = button_seen = estimator_ready = 0;
    ground_tick = state_tick = SysTick_count;
    landing_enabled = alarm_enabled = airborne = 0;
}

uint8_t AutoFlight_Active(void)
{
    return auto_flight.state != AUTO_IDLE;
}
uint8_t AutoFlight_HorizontalInput(void)
{
    return auto_flight.state == AUTO_HOVER && !LightTask_RemoteLocked();
}
uint8_t AutoFlight_MotorReady(void)
{
    return AutoFlight_Active() && ALL_flag.unlock;
}

/* Only called after packet checksum/header/channel validation. */
void AutoFlight_OnRcFrame(void)
{
    uint8_t aux1 = Remote.AUX1 > 1500U;
    uint8_t level;
    last_rc_tick = SysTick_count;
    rc_seen = 1;
    if (Remote.AUX3 > 1800U) level = 1;
    else if (Remote.AUX3 < 1200U) level = 0;
    else return;
    if (!button_seen) {
        button_level = button_candidate = level;
        button_tick = SysTick_count;
        last_aux1 = aux1;
        button_seen = 1;
    }
    if (level != button_candidate) {
        button_tick = SysTick_count;
        button_candidate = level;
    }
    if (aux1 != last_aux1 && AutoFlight_Active() && !LightTask_RemoteLocked())
        auto_flight.takeover_pending = 1;
    last_aux1 = aux1;
}

static void update_estimator(float dt)
{
    float sample, filtered, alpha;
    if (!range_fresh()) return;
    sample = (float)VL53L01_high * cosf(Angle.roll * 0.017453293f)
                                      * cosf(Angle.pitch * 0.017453293f);
    if (!estimator_ready ||
        (uint32_t)(SysTick_count - estimator_tick) > AUTO_RANGE_GRACE_TICKS) {
        history[0] = history[1] = history[2] = sample;
        history_index = 0;
        auto_flight.height_mm = old_height = sample;
        auto_flight.velocity_mm_s = 0.0f;
        estimator_ready = 1;
    }
    estimator_tick = SysTick_count;
    history[history_index] = sample;
    history_index = (uint8_t)((history_index + 1U) % 3U);
    filtered = median3(history[0], history[1], history[2]);
    alpha = dt / (0.045f + dt);
    auto_flight.height_mm += alpha * (filtered - auto_flight.height_mm);
    alpha = dt / (0.10f + dt);
    auto_flight.velocity_mm_s += alpha *
        ((auto_flight.height_mm - old_height) / dt - auto_flight.velocity_mm_s);
    old_height = auto_flight.height_mm;
}

static void start(void)
{
    float h = auto_flight.height_mm;
    ++auto_flight.start_request_id;
    auto_flight.error = AUTO_OK;
    auto_flight.alarm = AUTO_ALARM_NONE;
    if (!rc_seen || (uint32_t)(SysTick_count - last_rc_tick) > AUTO_RC_START_TICKS) {
        auto_flight.error = AUTO_REJECT_RC; return;
    }
    if (Remote.thr > 1100U || absf((float)Remote.roll - 1500.0f) > 100.0f ||
        absf((float)Remote.pitch - 1500.0f) > 100.0f ||
        absf((float)Remote.yaw - 1500.0f) > 100.0f) {
        auto_flight.error = AUTO_REJECT_STICKS; return;
    }
    if (!range_fresh() || flow_valid_samples < 40U) {
        auto_flight.error = AUTO_REJECT_HEIGHT;
        auto_flight.alarm = AUTO_ALARM_SENSOR; return;
    }
    if (absf(Angle.roll) > 10.0f || absf(Angle.pitch) > 10.0f) {
        auto_flight.error = AUTO_REJECT_TILT; return;
    }
    if (h > 250.0f || h < (float)FLOW_HEIGHT_MIN_MM ||
        (uint32_t)(SysTick_count - ground_tick) < 167U) {
        auto_flight.error = AUTO_REJECT_GROUND; return;
    }
    ground_height = h;
    ground_reference = h;
    ground_tick = state_tick = SysTick_count;
    auto_flight.state = AUTO_SPOOL;
    auto_flight.setpoint_mm = h;
    auto_flight.throttle = 0.0f;
    auto_flight.stable_seconds = 0.0f;
    auto_flight.takeover_pending = 0;
    ramp_seconds = flow_lost_seconds = landing_gesture_seconds = 0.0f;
    landing_arm_seconds = 0.0f;
    low_seconds = high_seconds = touchdown_seconds = 0.0f;
    landing_enabled = alarm_enabled = airborne = 0;
    reset_height_pid();
    Remote_ArmForAutoFlight();
    ALL_flag.height_lock = ALL_flag.take_off = 1;
    ALL_flag.flow_control = ALL_flag.take_down = 0;
    /* Publish unlock last: the next 3 ms attitude cycle resets its own PID. */
    ALL_flag.unlock = 1;
}

static void update_alarm(float dt)
{
    float h = auto_flight.height_mm;
    if (!range_fresh()) {
        low_seconds = high_seconds = 0.0f;
        auto_flight.alarm = AUTO_ALARM_SENSOR;
        return;
    }
    if (auto_flight.state == AUTO_LANDING) return;
    if (!alarm_enabled) {
        auto_flight.alarm = AUTO_ALARM_NONE;
        return;
    }
    low_seconds = h < AUTO_ALARM_LOW_MM ? low_seconds + dt : 0.0f;
    high_seconds = h > AUTO_ALARM_HIGH_MM ? high_seconds + dt : 0.0f;
    if (low_seconds >= 0.15f) auto_flight.alarm = AUTO_ALARM_LOW;
    else if (high_seconds >= 0.15f) auto_flight.alarm = AUTO_ALARM_HIGH;
    else if (h >= AUTO_ALARM_LOW_MM + AUTO_ALARM_HYST_MM &&
             h <= AUTO_ALARM_HIGH_MM - AUTO_ALARM_HYST_MM)
        auto_flight.alarm = AUTO_ALARM_NONE;
}

void AutoFlight_Tick(float dt)
{
    uint8_t button_event = 0;
    float manual_throttle;
    if (dt <= 0.0f) return;
    if (rc_seen && (uint32_t)(SysTick_count - last_rc_tick) > AUTO_RC_LOSS_TICKS) {
        ALL_flag.unlock = 0;
        if (AutoFlight_Active()) {
            stop_auto();
            auto_flight.error = AUTO_ERROR_RC_LOSS;
            auto_flight.alarm = AUTO_ALARM_SENSOR;
        }
    }
    update_estimator(dt);
    if (button_seen && button_candidate != button_level &&
        (uint32_t)(SysTick_count - button_tick) >= AUTO_BUTTON_TICKS &&
        (uint32_t)(SysTick_count - last_rc_tick) <= AUTO_RC_START_TICKS) {
        button_level = button_candidate;
        button_event = AUTO_AUX3_LATCHED || button_level;
    }
    if (!AutoFlight_Active()) {
        if (!range_fresh() || absf(auto_flight.height_mm - ground_reference) > 10.0f ||
            absf(auto_flight.velocity_mm_s) > 40.0f ||
            absf(Angle.roll) > 10.0f || absf(Angle.pitch) > 10.0f) {
            ground_reference = auto_flight.height_mm;
            ground_tick = SysTick_count;
        }
        /* Ground/range/stick checks in start() also apply after manual arming.
         * Never restart from the air or interrupt a light-source hold. */
        if (button_event && !LightTask_RemoteLocked()) start();
        return;
    }
    if (!ALL_flag.unlock) {
        stop_auto();
        auto_flight.alarm = AUTO_ALARM_NONE;
        return;
    }
    /* Emergency low-throttle/left-yaw lock always overrides automatic flight. */
    if (Remote.thr < 1050U && Remote.yaw < 1150U) {
        ALL_flag.unlock = 0;
        stop_auto();
        return;
    }
    if (auto_flight.height_mm > ground_height + 100.0f) airborne = 1;
    if ((uint32_t)(SysTick_count - flow_last_valid_tick) > AUTO_RANGE_GRACE_TICKS) {
        if (!airborne) {
            ALL_flag.unlock = 0;
            stop_auto();
        } else {
            /* No trustworthy altitude remains. Hold the last collective until
             * explicit throttle-matched manual takeover or emergency lock.
             * Never silently substitute a minimum throttle from the RC stick. */
            auto_flight.state = AUTO_RANGE_FAULT;
            auto_flight.takeover_pending = 1;
            ALL_flag.height_lock = ALL_flag.flow_control = 0;
        }
        auto_flight.error = AUTO_ERROR_RANGE;
        auto_flight.alarm = AUTO_ALARM_SENSOR;
    }
    manual_throttle = 200.0f + 0.45f * ((float)Remote.thr - 1000.0f);
    if ((!LightTask_RemoteLocked() || auto_flight.state != AUTO_HOVER) &&
        auto_flight.takeover_pending && Remote.thr > 1150U &&
        absf(manual_throttle - auto_flight.throttle) <= 40.0f) {
        stop_auto();
        auto_flight.alarm = AUTO_ALARM_NONE;
        return;
    }
    if (auto_flight.state == AUTO_RANGE_FAULT) {
        pidRoll.desired = pidPitch.desired = 0.0f;
        return;
    }
    if (!AutoFlight_Active()) return;
    ALL_flag.height_lock = 1;
    ALL_flag.flow_control = auto_flight.height_mm > 200.0f && range_fresh() &&
        flow_data.squal > 30U;
    if (!ALL_flag.flow_control) pidRoll.desired = pidPitch.desired = 0.0f;
    if (auto_flight.height_mm > 200.0f && auto_flight.state != AUTO_LANDING) {
        if (ALL_flag.flow_control) flow_lost_seconds = 0.0f;
        else flow_lost_seconds += dt;
        if (flow_lost_seconds > AUTO_FLOW_LOSS_SECONDS) land(AUTO_ERROR_FLOW);
    }
    if (LightTask_RemoteLocked()) {
        landing_arm_seconds = landing_gesture_seconds = 0.0f;
        landing_enabled = 0;
        auto_flight.takeover_pending = 0;
    } else {
    if (Remote.thr >= 1250U && Remote.thr <= 1750U) landing_arm_seconds += dt;
    else landing_arm_seconds = 0.0f;
    if (landing_arm_seconds >= 0.25f) landing_enabled = 1;
    if (landing_enabled && Remote.thr < 1100U && !auto_flight.takeover_pending)
        landing_gesture_seconds += dt;
    else landing_gesture_seconds = 0.0f;
    if (landing_gesture_seconds > 0.15f) land(AUTO_OK);
    }
    if (auto_flight.state == AUTO_SPOOL) {
        ramp_seconds += dt;
        auto_flight.throttle = AUTO_IDLE_THROTTLE *
            LIMIT(ramp_seconds / AUTO_SPOOL_SECONDS, 0.0f, 1.0f);
        if (ramp_seconds >= AUTO_SPOOL_SECONDS) {
            auto_flight.state = AUTO_CLIMB;
            state_tick = SysTick_count;
            ramp_seconds = 0.0f;
        }
        return;
    }
    if (!range_fresh()) {
        auto_flight.stable_seconds = 0.0f;
        update_alarm(dt);
        return; /* Brief bad frames hold the previous output, never integrate. */
    }
    if (auto_flight.state == AUTO_CLIMB || auto_flight.state == AUTO_HOVER) {
        auto_flight.setpoint_mm += AUTO_CLIMB_RATE_MM_S * dt;
        if (auto_flight.setpoint_mm > AUTO_TARGET_HEIGHT_MM)
            auto_flight.setpoint_mm = AUTO_TARGET_HEIGHT_MM;
        if (auto_flight.height_mm >= AUTO_ALARM_LOW_MM) alarm_enabled = 1;
        if (auto_flight.setpoint_mm >= AUTO_TARGET_HEIGHT_MM &&
            absf(auto_flight.height_mm - AUTO_TARGET_HEIGHT_MM) <= AUTO_HEIGHT_TOLERANCE_MM &&
            absf(auto_flight.velocity_mm_s) <= 80.0f && ALL_flag.flow_control)
            auto_flight.stable_seconds += dt;
        else auto_flight.stable_seconds = 0.0f;
        if (auto_flight.stable_seconds >= AUTO_STABLE_SECONDS)
            auto_flight.state = AUTO_HOVER;
        if (!airborne && (uint32_t)(SysTick_count - state_tick) > 1667U) {
            ALL_flag.unlock = 0;
            stop_auto();
            auto_flight.error = AUTO_ERROR_NO_LIFTOFF;
            auto_flight.alarm = AUTO_ALARM_SENSOR;
            return;
        }
        if (auto_flight.state == AUTO_CLIMB &&
            (uint32_t)(SysTick_count - state_tick) > 5000U)
            land(AUTO_ERROR_CLIMB_TIMEOUT);
    }
    if (auto_flight.state == AUTO_LANDING) {
        auto_flight.setpoint_mm -= AUTO_DESCENT_RATE_MM_S * dt;
        if (auto_flight.setpoint_mm < ground_height - 20.0f)
            auto_flight.setpoint_mm = ground_height - 20.0f;
        if (auto_flight.height_mm <= ground_height + 25.0f &&
            absf(auto_flight.velocity_mm_s) < 60.0f) touchdown_seconds += dt;
        else touchdown_seconds = 0.0f;
        if (touchdown_seconds >= 0.4f) {
            ALL_flag.unlock = 0;
            stop_auto();
            auto_flight.alarm = AUTO_ALARM_NONE;
            return;
        }
    }
    update_alarm(dt);
}

/* Retains the supplier's height/velocity cascade and calibrated gains.
 * Derivative acts on measured velocity, with bounded integral and collective. */
void AutoFlight_HeightControl(float dt)
{
    float desired_velocity, error, derivative, proposed_integral, output, base;
    float old_integral;
    if (auto_flight.state == AUTO_SPOOL || auto_flight.state == AUTO_RANGE_FAULT ||
        !range_fresh()) return;
    if (dt <= 0.0f) return;
    pidHeightHigh.measured = auto_flight.height_mm;
    pidHeightHigh.desired = auto_flight.setpoint_mm;
    pidHeightHigh.out = LIMIT(pidHeightHigh.kp *
        (pidHeightHigh.desired - pidHeightHigh.measured),
        -AUTO_DESCENT_RATE_MM_S, AUTO_CLIMB_RATE_MM_S);
    desired_velocity = pidHeightHigh.out;
    pidHeightRate.desired += LIMIT(desired_velocity - pidHeightRate.desired,
        -AUTO_VELOCITY_ACCEL * dt, AUTO_VELOCITY_ACCEL * dt);
    pidHeightRate.measured = auto_flight.velocity_mm_s;
    error = pidHeightRate.desired - pidHeightRate.measured;
    derivative = -(pidHeightRate.measured - old_velocity) / dt;
    old_velocity = pidHeightRate.measured;
    old_integral = pidHeightRate.integ;
    proposed_integral = LIMIT(old_integral + error * dt, -2500.0f, 2500.0f);
    ramp_seconds += dt;
    base = AUTO_IDLE_THROTTLE + (AUTO_HOVER_THROTTLE - AUTO_IDLE_THROTTLE) *
        LIMIT(ramp_seconds / 1.0f, 0.0f, 1.0f);
    output = base + pidHeightRate.kp * error + pidHeightRate.ki * proposed_integral +
        pidHeightRate.kd * derivative;
    if (!((output > AUTO_MAX_THROTTLE && error > 0.0f) ||
          (output < 0.0f && error < 0.0f))) pidHeightRate.integ = proposed_integral;
    pidHeightRate.out = output - base;
    auto_flight.throttle = LIMIT(output, 0.0f, AUTO_MAX_THROTTLE);
}
