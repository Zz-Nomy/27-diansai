#ifndef LIGHT_TASK_CONFIG_H
#define LIGHT_TASK_CONFIG_H

/* Initial tuning values, NOT measured lamp specifications.
 * Assumption: 5 W -> 500 lm, upward Lambertian emitter at 0.8 m:
 * E = 500 / (pi * 0.8^2) = 249 lx above the ambient baseline.
 * Replace after calibration with the actual lamp and sensor mounting. */
#ifndef LIGHT_ARRIVAL_DELTA_LUX
#define LIGHT_ARRIVAL_DELTA_LUX       250.0f
#endif
#define LIGHT_THRESHOLD_CALIBRATED   0U
#define LIGHT_BASELINE_SAMPLES       5U
#define LIGHT_RISE_MIN_LUX           20.0f
#define LIGHT_RISE_RATIO             0.05f
#define LIGHT_RISE_CONFIRM_SAMPLES   2U
#define LIGHT_ARRIVAL_CONFIRM_SAMPLES 3U
#define LIGHT_ALERT_DURATION_MS      3000U
#define LIGHT_BEEP_PERIOD_MS         1000U
#define LIGHT_BEEP_ON_MS             150U
#define LIGHT_LED_HALF_PERIOD_MS     200U
#define LIGHT_HOLD_SECONDS           3.0f
#define LIGHT_HOLD_TIMEOUT_SECONDS   12.0f
#define LIGHT_SENSOR_GRACE_SECONDS   1.0f
#define LIGHT_MAX_TILT_DEG           8.0f
/* Supplier optical-flow units, not metres or metres/second. */
#define LIGHT_STABLE_FLOW_SPEED      1.0f
#define LIGHT_HOLD_POSITION_ERROR    5.0f
#define BH1750_ADDRESS_7BIT          0x23U
#define BH1750_SAMPLE_SECONDS        0.204f
#define BH1750_RETRY_SECONDS         1.0f
#define BH1750_MAX_AGE_SECONDS       0.45f

#endif
