#ifndef AUTO_FLIGHT_H
#define AUTO_FLIGHT_H
#include "ALL_DATA.h"
#include "auto_flight_config.h"
#include "open_takeoff_config.h"
typedef enum {
    AUTO_IDLE=0, AUTO_SPOOL=1, AUTO_THR_HOLD=2,
    AUTO_TAKEOVER=3, AUTO_MANUAL=4, AUTO_RC_FAULT=5
} AutoFlightState;
typedef enum {
    AUTO_OK=0, AUTO_REJECT_RC=1, AUTO_REJECT_STICKS=2,
    AUTO_REJECT_HEIGHT=3, AUTO_REJECT_GROUND=4, AUTO_REJECT_TILT=5,
    AUTO_ERROR_RANGE=6, AUTO_ERROR_FLOW=7, AUTO_ERROR_NO_LIFTOFF=8,
    AUTO_ERROR_CLIMB_TIMEOUT=9, AUTO_ERROR_RC_LOSS=10
} AutoFlightError;
typedef enum { AUTO_ALARM_NONE=0, AUTO_ALARM_LOW, AUTO_ALARM_HIGH, AUTO_ALARM_SENSOR } AutoFlightAlarm;
typedef struct {
    AutoFlightState state;
    AutoFlightError error;
    AutoFlightAlarm alarm;
    float height_mm, velocity_mm_s, setpoint_mm, throttle;
    float stable_seconds;
    uint16_t start_request_id;
    uint8_t takeover_pending;
} AutoFlightStatus;
extern volatile AutoFlightStatus auto_flight;
void AutoFlight_Init(void);
void AutoFlight_OnRcFrame(void);
void AutoFlight_Tick(float dt);
void AutoFlight_EmergencyStop(AutoFlightError error);
float AutoFlight_ManualThrottle(void);
uint8_t AutoFlight_Active(void);
uint8_t AutoFlight_HorizontalInput(void);
uint8_t AutoFlight_MotorReady(void);
void AutoFlight_HeightControl(float dt);
#endif
