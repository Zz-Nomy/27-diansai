#include "light_task.h"
#include "bh1750.h"
#include <string.h>
volatile LightTaskStatus light_task;
void LightTask_Init(void) { memset((void *)&light_task,0,sizeof(light_task)); }
void LightTask_Tick(float dt) { (void)dt; light_task.sensor_fault=!bh1750.valid; }
uint8_t LightTask_RemoteLocked(void) { return 0; }
uint8_t LightTask_AlertActive(void) { return 0; }
uint8_t LightTask_AlertLedOn(void) { return 0; }
uint16_t LightTask_AlertElapsedMs(void) { return 3000U; }
