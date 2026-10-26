#ifndef LIGHT_TASK_H
#define LIGHT_TASK_H
#include "stm32f10x.h"
#include "light_task_config.h"

typedef enum {
    LIGHT_WAIT_FLIGHT = 0, LIGHT_BASELINE, LIGHT_WAIT_RISE,
    LIGHT_MANUAL_APPROACH, LIGHT_HOLD, LIGHT_COMPLETE, LIGHT_ABORTED
} LightTaskState;

typedef struct {
    LightTaskState state;
    float ambient_lux, arrival_lux, delta_lux;
    float hold_x, hold_y, hold_height_mm, stable_seconds;
    uint32_t event_id;
    uint8_t source_detected, sensor_fault, threshold_calibrated;
} LightTaskStatus;

extern volatile LightTaskStatus light_task;
void LightTask_Init(void);
void LightTask_Tick(float dt);
uint8_t LightTask_RemoteLocked(void);
uint8_t LightTask_AlertActive(void);
uint8_t LightTask_AlertLedOn(void);
uint16_t LightTask_AlertElapsedMs(void);
#endif
