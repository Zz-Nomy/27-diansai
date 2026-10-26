#include "stm32f10x.h"
#include "auto_flight.h"
#include "remote.h"
#include "control.h"
#include "myMath.h"
#include <string.h>

volatile AutoFlightStatus auto_flight;
static volatile uint32_t last_rc_tick;
static volatile uint8_t rc_seen, buttons_seen, pending_start, pending_takeover;
static uint8_t aux2_level, aux3_level, aux4_level;
static float ramp_seconds, match_seconds;
static float abs_value(float x) { return x<0.0f ? -x : x; }
static uint8_t button_level(uint16_t v) { return v>1500U; }

/* Also called directly from a validated RC frame: AUX2 cuts PWM immediately,
 * before waiting for the 6 ms task. A held/toggled level cannot re-arm. */
void AutoFlight_EmergencyStop(AutoFlightError error)
{
    ALL_flag.unlock=0;
    ALL_flag.height_lock=ALL_flag.flow_control=0;
    ALL_flag.take_off=ALL_flag.take_down=0;
    pending_start=pending_takeover=0;
    ramp_seconds=match_seconds=0.0f;
    auto_flight.throttle=0.0f;
    auto_flight.takeover_pending=0;
    auto_flight.stable_seconds=0.0f;
    auto_flight.error=error;
    auto_flight.alarm=error==AUTO_ERROR_RC_LOSS ? AUTO_ALARM_SENSOR : AUTO_ALARM_NONE;
    auto_flight.state=error==AUTO_ERROR_RC_LOSS ? AUTO_RC_FAULT : AUTO_IDLE;
    motor_PWM_Value[0]=motor_PWM_Value[1]=motor_PWM_Value[2]=motor_PWM_Value[3]=0;
    TIM2->CCR1=TIM2->CCR2=TIM2->CCR3=TIM2->CCR4=0;
}
void AutoFlight_Init(void)
{
    memset((void *)&auto_flight,0,sizeof(auto_flight));
    rc_seen=buttons_seen=pending_start=pending_takeover=0;
    last_rc_tick=SysTick_count;
    AutoFlight_EmergencyStop(AUTO_OK);
}
uint8_t AutoFlight_Active(void)
{
    return auto_flight.state==AUTO_SPOOL || auto_flight.state==AUTO_THR_HOLD ||
           auto_flight.state==AUTO_TAKEOVER;
}
uint8_t AutoFlight_HorizontalInput(void) { return auto_flight.state==AUTO_MANUAL; }
uint8_t AutoFlight_MotorReady(void) { return AutoFlight_Active() && ALL_flag.unlock; }
float AutoFlight_ManualThrottle(void)
{
    if (Remote.thr<=1010U) return 0.0f;
    /* Smooth the supplier's 200-count low-stick discontinuity for takeover. */
    if (Remote.thr<1100U) return 2.45f*((float)Remote.thr-1000.0f);
    return LIMIT(200.0f+0.45f*((float)Remote.thr-1000.0f),0.0f,OPEN_TAKEOFF_MAX_PWM);
}
void AutoFlight_OnRcFrame(void)
{
    uint8_t a2=button_level(Remote.AUX2),a3=button_level(Remote.AUX3);
    uint8_t a4=button_level(Remote.AUX4),e2,e3,e4;
    last_rc_tick=SysTick_count; rc_seen=1;
    if (!buttons_seen) {
        aux2_level=a2; aux3_level=a3; aux4_level=a4; buttons_seen=1;
        return; /* Startup levels are not button presses. */
    }
    e2=a2!=aux2_level; e3=a3!=aux3_level; e4=a4!=aux4_level;
    aux2_level=a2; aux3_level=a3; aux4_level=a4;
    if (e2) { AutoFlight_EmergencyStop(AUTO_OK); return; }
    if (e4) {
        pending_start=0;
        if (AutoFlight_Active()) pending_takeover=1;
        return;
    }
    if (e3) pending_start=1;
}
static void start_open_takeoff(void)
{
    ++auto_flight.start_request_id;
    if (AutoFlight_Active() || auto_flight.state==AUTO_MANUAL) {
        auto_flight.error=AUTO_REJECT_GROUND; return;
    }
    if (!rc_seen || (uint32_t)(SysTick_count-last_rc_tick)>OPEN_RC_START_TICKS) {
        auto_flight.error=AUTO_REJECT_RC; return;
    }
    if (Remote.thr>1100U || abs_value((float)Remote.roll-1500.0f)>100.0f ||
        abs_value((float)Remote.pitch-1500.0f)>100.0f ||
        abs_value((float)Remote.yaw-1500.0f)>100.0f) {
        auto_flight.error=AUTO_REJECT_STICKS; return;
    }
    if (abs_value(Angle.roll)>OPEN_START_TILT_DEG ||
        abs_value(Angle.pitch)>OPEN_START_TILT_DEG) {
        auto_flight.error=AUTO_REJECT_TILT; return;
    }
    Remote_ArmForAutoFlight();
    ALL_flag.height_lock=ALL_flag.flow_control=0;
    ALL_flag.take_off=ALL_flag.take_down=0;
    auto_flight.state=AUTO_SPOOL;
    auto_flight.error=AUTO_OK; auto_flight.alarm=AUTO_ALARM_NONE;
    auto_flight.throttle=auto_flight.stable_seconds=0.0f;
    auto_flight.takeover_pending=0;
    ramp_seconds=match_seconds=0.0f;
    ALL_flag.unlock=1;
}
void AutoFlight_Tick(float dt)
{
    uint8_t start_request,takeover_request;
    uint32_t mask;
    if (dt<=0.0f) return;
    if (rc_seen && (uint32_t)(SysTick_count-last_rc_tick)>OPEN_RC_LOSS_TICKS) {
        if (ALL_flag.unlock || AutoFlight_Active() || auto_flight.state==AUTO_MANUAL)
            AutoFlight_EmergencyStop(AUTO_ERROR_RC_LOSS);
    }
    if (ALL_flag.unlock &&
        (abs_value(Angle.roll)>OPEN_EMERGENCY_TILT_DEG ||
         abs_value(Angle.pitch)>OPEN_EMERGENCY_TILT_DEG)) {
        AutoFlight_EmergencyStop(AUTO_REJECT_TILT); return;
    }
    if (ALL_flag.unlock && Remote.thr<1050U && Remote.yaw<1150U) {
        AutoFlight_EmergencyStop(AUTO_OK); return;
    }
    mask=__get_PRIMASK(); __disable_irq();
    start_request=pending_start; takeover_request=pending_takeover;
    pending_start=pending_takeover=0;
    __set_PRIMASK(mask);
    if (start_request) start_open_takeoff();
    if (takeover_request && AutoFlight_Active()) {
        auto_flight.state=AUTO_TAKEOVER;
        auto_flight.takeover_pending=1;
        match_seconds=0.0f;
    }
    if (!ALL_flag.unlock) {
        if (AutoFlight_Active() || auto_flight.state==AUTO_MANUAL)
            AutoFlight_EmergencyStop(AUTO_OK);
        return;
    }
    ALL_flag.height_lock=ALL_flag.flow_control=0;
    if (auto_flight.state==AUTO_SPOOL) {
        ramp_seconds+=dt;
        auto_flight.throttle=LIMIT(OPEN_TAKEOFF_TARGET_PWM,0.0f,OPEN_TAKEOFF_MAX_PWM)*
            LIMIT(ramp_seconds/OPEN_TAKEOFF_RAMP_SECONDS,0.0f,1.0f);
        if (ramp_seconds>=OPEN_TAKEOFF_RAMP_SECONDS) auto_flight.state=AUTO_THR_HOLD;
    } else if (auto_flight.state==AUTO_TAKEOVER) {
        if (abs_value(AutoFlight_ManualThrottle()-auto_flight.throttle)<=OPEN_TAKEOVER_MATCH_PWM)
            match_seconds+=dt;
        else match_seconds=0.0f;
        auto_flight.stable_seconds=match_seconds;
        if (match_seconds>=OPEN_TAKEOVER_MATCH_SECONDS) {
            auto_flight.state=AUTO_MANUAL;
            auto_flight.takeover_pending=0;
        }
    }
}
/* No range input, altitude estimate, vertical PID or position hold in this version. */
void AutoFlight_HeightControl(float dt) { (void)dt; }
