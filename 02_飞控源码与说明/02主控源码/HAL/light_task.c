#include "ALL_DEFINE.h"
#include "auto_flight.h"
#include "flow.h"
#include "bh1750.h"
#include "light_task.h"
#include <string.h>

volatile LightTaskStatus light_task;
static uint32_t last_sample_id, alert_tick;
static uint8_t baseline_count, rise_count, arrival_count;
static uint8_t flight_seen, alert_running, previous_stable;
static float baseline_sum, previous_lux, rise_reference;
static float hold_seconds, bad_sensor_seconds;

static float light_abs(float x) { return x < 0.0f ? -x : x; }
static uint8_t light_fresh(void)
{
    return bh1750.valid && bh1750.age_seconds <= BH1750_MAX_AGE_SECONDS;
}
static uint8_t light_position_valid(void)
{
    return ALL_flag.unlock &&
        (auto_flight.state == AUTO_HOVER || auto_flight.state == AUTO_IDLE) &&
        ALL_flag.height_lock && ALL_flag.flow_control && flow_height_valid &&
        (uint32_t)(SysTick_count - flow_last_valid_tick) <= 4U &&
        flow_data.squal > 30U;
}
static uint8_t light_pose_stable(void)
{
    /* Detect light after either manual or automatic takeoff. Detection does
     * not enable any flight controller or claim position-hold capability. */
    return ALL_flag.unlock && flow_height_valid &&
        (uint32_t)(SysTick_count - flow_last_valid_tick) <= 4U &&
        light_abs(auto_flight.height_mm - AUTO_TARGET_HEIGHT_MM) <= AUTO_HEIGHT_TOLERANCE_MM &&
        light_abs(auto_flight.velocity_mm_s) <= 80.0f &&
        light_abs(Angle.roll) <= LIGHT_MAX_TILT_DEG &&
        light_abs(Angle.pitch) <= LIGHT_MAX_TILT_DEG &&
        (flow_data.squal <= 30U ||
         (light_abs(flow_x_vel_lpf_i) <= LIGHT_STABLE_FLOW_SPEED &&
          light_abs(flow_y_vel_lpf_i) <= LIGHT_STABLE_FLOW_SPEED));
}
static void light_reset_baseline(void)
{
    baseline_count = rise_count = arrival_count = previous_stable = 0;
    baseline_sum = previous_lux = 0.0f;
    light_task.state = LIGHT_BASELINE;
}
void LightTask_Init(void)
{
    memset((void *)&light_task, 0, sizeof(light_task));
    light_task.threshold_calibrated = LIGHT_THRESHOLD_CALIBRATED;
    light_task.delta_lux = LIGHT_ARRIVAL_DELTA_LUX;
    light_reset_baseline();
    light_task.state = LIGHT_WAIT_FLIGHT;
    flight_seen = alert_running = 0;
    last_sample_id = bh1750.sample_id;
    hold_seconds = bad_sensor_seconds = 0.0f;
}
uint8_t LightTask_RemoteLocked(void)
{
    return light_task.state == LIGHT_HOLD;
}
uint16_t LightTask_AlertElapsedMs(void)
{
    uint32_t ticks = (uint32_t)(SysTick_count - alert_tick);
    if (!alert_running || ticks >= (LIGHT_ALERT_DURATION_MS + 2U) / 3U)
        return LIGHT_ALERT_DURATION_MS;
    return (uint16_t)(ticks * 3U);
}
uint8_t LightTask_AlertActive(void)
{
    return LightTask_AlertElapsedMs() < LIGHT_ALERT_DURATION_MS;
}
uint8_t LightTask_AlertLedOn(void)
{
    return LightTask_AlertActive() &&
        ((LightTask_AlertElapsedMs() / LIGHT_LED_HALF_PERIOD_MS) & 1U) == 0U;
}

void LightTask_Tick(float dt)
{
    uint8_t new_sample, stable;
    float lux, minimum_rise;
    if (dt <= 0.0f) return;
    if (!LightTask_AlertActive()) alert_running = 0;
    light_task.sensor_fault = !light_fresh();
    new_sample = bh1750.sample_id != last_sample_id;
    last_sample_id = bh1750.sample_id;
    if (!ALL_flag.unlock) {
        flight_seen = alert_running = 0;
        light_task.state = LIGHT_WAIT_FLIGHT;
        light_task.source_detected = 0;
        light_task.stable_seconds = 0.0f;
        return;
    }
    if (light_task.state == LIGHT_HOLD) {
        hold_seconds += dt;
        if (!light_position_valid()) {
            light_task.stable_seconds = 0.0f;
            bad_sensor_seconds += dt;
        } else if (!light_fresh() || bh1750.lux < light_task.arrival_lux) {
            light_task.stable_seconds = 0.0f;
            bad_sensor_seconds += dt;
        } else {
            bad_sensor_seconds = 0.0f;
            stable = light_pose_stable() &&
                light_abs(flow_x_lpf_att_i - light_task.hold_x) <= LIGHT_HOLD_POSITION_ERROR &&
                light_abs(flow_y_lpf_att_i - light_task.hold_y) <= LIGHT_HOLD_POSITION_ERROR;
            if (stable) light_task.stable_seconds += dt;
            else light_task.stable_seconds = 0.0f;
        }
        if ((AutoFlight_Active() && auto_flight.state != AUTO_HOVER) ||
            bad_sensor_seconds >= LIGHT_SENSOR_GRACE_SECONDS ||
            hold_seconds >= LIGHT_HOLD_TIMEOUT_SECONDS) {
            light_task.state = LIGHT_ABORTED;
            light_task.stable_seconds = 0.0f;
        } else if (light_task.stable_seconds + 0.00001f >= LIGHT_HOLD_SECONDS) {
            light_task.state = LIGHT_COMPLETE;
            light_task.stable_seconds = LIGHT_HOLD_SECONDS;
        }
        return;
    }
    if (light_task.state == LIGHT_COMPLETE || light_task.state == LIGHT_ABORTED) return;
    if (AutoFlight_Active() && auto_flight.state != AUTO_HOVER) {
        if (flight_seen) light_task.state = LIGHT_ABORTED;
        return;
    }
    if (!flight_seen) {
        if (!light_pose_stable()) return;
        light_reset_baseline();
        light_task.source_detected = 0;
        flight_seen = 1;
    }
    if (!light_fresh()) {
        arrival_count = 0;
        if (light_task.state == LIGHT_BASELINE || light_task.state == LIGHT_WAIT_RISE)
            light_reset_baseline();
        return;
    }
    if (!new_sample) return;
    lux = bh1750.lux;
    stable = light_pose_stable();
    if (light_task.state == LIGHT_BASELINE) {
        if (!stable) { baseline_count = 0; baseline_sum = 0.0f; return; }
        /* Restart if the lamp turns on while collecting the ambient baseline. */
        if (baseline_count && light_abs(lux - baseline_sum / baseline_count) >
            LIGHT_RISE_MIN_LUX + lux * LIGHT_RISE_RATIO) {
            baseline_count = 0;
            baseline_sum = 0.0f;
        }
        baseline_sum += lux;
        if (++baseline_count >= LIGHT_BASELINE_SAMPLES) {
            light_task.ambient_lux = baseline_sum / baseline_count;
            light_task.arrival_lux = light_task.ambient_lux + light_task.delta_lux;
            previous_lux = lux;
            previous_stable = 1;
            light_task.state = LIGHT_WAIT_RISE;
        }
        return;
    }
    if (light_task.state == LIGHT_WAIT_RISE) {
        minimum_rise = light_task.ambient_lux * LIGHT_RISE_RATIO;
        if (minimum_rise < LIGHT_RISE_MIN_LUX) minimum_rise = LIGHT_RISE_MIN_LUX;
        if (!stable || !previous_stable) rise_count = 0;
        else if (rise_count) {
            if (lux - rise_reference >= minimum_rise) ++rise_count;
            else rise_count = 0;
        } else if (lux - previous_lux >= minimum_rise) {
            rise_reference = previous_lux;
            rise_count = 1;
        }
        if (rise_count >= LIGHT_RISE_CONFIRM_SAMPLES) {
            light_task.source_detected = 1;
            ++light_task.event_id;
            alert_tick = SysTick_count;
            alert_running = 1;
            light_task.state = LIGHT_MANUAL_APPROACH;
            arrival_count = 0;
        }
        previous_stable = stable;
        previous_lux = lux;
        return;
    }
    /* Confirmation uses independent completed measurements, not 6 ms reads of
     * the same conversion. Keep manual horizontal control while approaching. */
    if (light_task.state == LIGHT_MANUAL_APPROACH) {
        if (light_position_valid() &&
            light_abs(auto_flight.height_mm - AUTO_TARGET_HEIGHT_MM) <= AUTO_HEIGHT_TOLERANCE_MM &&
            light_abs(Angle.roll) <= LIGHT_MAX_TILT_DEG &&
            light_abs(Angle.pitch) <= LIGHT_MAX_TILT_DEG &&
            light_task.delta_lux > 0.0f && lux >= light_task.arrival_lux)
            ++arrival_count;
        else arrival_count = 0;
        if (arrival_count >= LIGHT_ARRIVAL_CONFIRM_SAMPLES) {
            light_task.hold_x = flow_x_lpf_att_i;
            light_task.hold_y = flow_y_lpf_att_i;
            light_task.hold_height_mm = auto_flight.height_mm;
            light_task.stable_seconds = hold_seconds = bad_sensor_seconds = 0.0f;
            light_task.state = LIGHT_HOLD;
            FlowControl_LatchPosition();
            /* Commands received during the lock are not queued for release. */
            auto_flight.takeover_pending = 0;
        }
    }
}
