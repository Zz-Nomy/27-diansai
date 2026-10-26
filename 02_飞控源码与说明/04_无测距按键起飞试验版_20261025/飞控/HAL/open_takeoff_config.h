#ifndef OPEN_TAKEOFF_CONFIG_H
#define OPEN_TAKEOFF_CONFIG_H
/* Motor PWM counts (0..1000), NOT altitude or calibrated lift thrust.
 * 500 is the supplier's former hover reference, not measured on this airframe. */
#define OPEN_TAKEOFF_TARGET_PWM       500.0f
#define OPEN_TAKEOFF_RAMP_SECONDS     3.0f
#define OPEN_TAKEOFF_MAX_PWM          750.0f
#define OPEN_TAKEOVER_MATCH_PWM       35.0f
#define OPEN_TAKEOVER_MATCH_SECONDS   0.25f
#define OPEN_RC_LOSS_TICKS            167U /* 501 ms, tick = 3 ms */
#define OPEN_RC_START_TICKS           80U
#define OPEN_START_TILT_DEG           10.0f
#define OPEN_EMERGENCY_TILT_DEG       50.0f
#endif
