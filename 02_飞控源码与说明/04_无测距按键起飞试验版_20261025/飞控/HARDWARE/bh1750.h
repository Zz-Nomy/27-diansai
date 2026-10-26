#ifndef BH1750_H
#define BH1750_H
#include "stm32f10x.h"

typedef struct {
    uint16_t raw;
    float lux;
    uint32_t sample_id;
    float age_seconds;
    uint16_t errors;
    uint8_t valid, saturated;
} BH1750Status;

extern volatile BH1750Status bh1750;
void BH1750_Init(void);
/* Run only in the flight scheduler, alongside MPU/laser bus accesses. */
void BH1750_Tick(float dt);
#endif
