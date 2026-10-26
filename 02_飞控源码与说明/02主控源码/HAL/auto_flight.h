#ifndef AUTO_FLIGHT_H
#define AUTO_FLIGHT_H
#include "ALL_DATA.h"
#include "auto_flight_config.h"

typedef enum {
    AUTO_IDLE = 0, AUTO_SPOOL, AUTO_CLIMB, AUTO_HOVER,
    AUTO_LANDING, AUTO_RANGE_FAULT
} AutoFlightState;

typedef enum {
    AUTO_OK = 0, AUTO_REJECT_RC, AUTO_REJECT_STICKS,
    AUTO_REJECT_HEIGHT, AUTO_REJECT_GROUND, AUTO_REJECT_TILT,
    AUTO_ERROR_RANGE, AUTO_ERROR_FLOW, AUTO_ERROR_NO_LIFTOFF,
    AUTO_ERROR_CLIMB_TIMEOUT, AUTO_ERROR_RC_LOSS
} AutoFlightError;

typedef enum {
    AUTO_ALARM_NONE = 0, AUTO_ALARM_LOW, AUTO_ALARM_HIGH,
    AUTO_ALARM_SENSOR
} AutoFlightAlarm;

typedef struct {
    AutoFlightState state;
    AutoFlightError error;
    AutoFlightAlarm alarm;
    float height_mm, velocity_mm_s, setpoint_mm, throttle;
    float stable_seconds;
    uint8_t takeover_pending;
} AutoFlightStatus;

extern volatile AutoFlightStatus auto_flight;
void AutoFlight_Init(void);
void AutoFlight_OnRcFrame(void);
void AutoFlight_Tick(float dt);
uint8_t AutoFlight_Active(void);
uint8_t AutoFlight_HorizontalInput(void);
uint8_t AutoFlight_MotorReady(void);
void AutoFlight_HeightControl(float dt);
#endif
