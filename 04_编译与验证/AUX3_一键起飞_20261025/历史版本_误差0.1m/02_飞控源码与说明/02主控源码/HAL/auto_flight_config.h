#ifndef AUTO_FLIGHT_CONFIG_H
#define AUTO_FLIGHT_CONFIG_H

/* Supplier I2C protocol image specifies centimetres. All control uses mm.
 * If a replacement actually reports mm, set this to 1 after a ruler check. */
#ifndef FLOW_HEIGHT_MM_PER_UNIT
#define FLOW_HEIGHT_MM_PER_UNIT 10U
#endif
#define FLOW_HEIGHT_MIN_MM       40U
#define FLOW_HEIGHT_MAX_MM       4000U

#define AUTO_TARGET_HEIGHT_MM    800.0f
#define AUTO_HEIGHT_TOLERANCE_MM 100.0f
#define AUTO_ALARM_LOW_MM        700.0f
#define AUTO_ALARM_HIGH_MM       900.0f
#define AUTO_ALARM_HYST_MM       20.0f
#define AUTO_CLIMB_RATE_MM_S     200.0f
#define AUTO_DESCENT_RATE_MM_S   150.0f
#define AUTO_VELOCITY_ACCEL      300.0f
#define AUTO_HOVER_THROTTLE      500.0f
#define AUTO_IDLE_THROTTLE       200.0f
#define AUTO_MAX_THROTTLE        750.0f
#define AUTO_SPOOL_SECONDS       0.8f
#define AUTO_STABLE_SECONDS      1.0f
#define AUTO_RANGE_GRACE_TICKS   40U  /* SysTick_count is a 3 ms counter. */
#define AUTO_RC_START_TICKS      80U
#define AUTO_RC_LOSS_TICKS       1000U
#define AUTO_BUTTON_TICKS        15U
#define AUTO_FLOW_LOSS_SECONDS   2.0f

/* AUX1 in the supplier transmitter toggles 1000/2000 on each press.
 * Default to the same behaviour for AUX3; set 0 for a momentary AUX3. */
#ifndef AUTO_AUX3_LATCHED
#define AUTO_AUX3_LATCHED 1
#endif

/* TIM3: front right PA6/CH1, front left PA7/CH2 are BLUE.
 * Rear PB0/CH3 and PB1/CH4 are RED. Active-low supplier LED outputs. */
#define AUTO_BLUE_LED_MASK 0x03U
#define AUTO_RED_LED_MASK  0x0CU
#endif
