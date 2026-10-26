#include "bh1750.h"
#include "I2C.h"
#include "light_task_config.h"
#include <string.h>

volatile BH1750Status bh1750;
static uint8_t init_index;
static float wait_seconds, retry_seconds;
/* Power on, data reset, explicitly restore MTreg=69, continuous H-resolution.
 * Each instruction is its own I2C transaction; there is no register address. */
static const uint8_t init_commands[] = {0x01, 0x07, 0x42, 0x65, 0x10};

void BH1750_Init(void)
{
    memset((void *)&bh1750, 0, sizeof(bh1750));
    bh1750.age_seconds = BH1750_MAX_AGE_SECONDS + 1.0f;
    init_index = 0;
    wait_seconds = retry_seconds = 0.0f;
}

static void bh1750_fail(void)
{
    bh1750.valid = 0;
    if (bh1750.errors != 65535U) ++bh1750.errors;
    init_index = 0;
    wait_seconds = 0.0f;
    retry_seconds = BH1750_RETRY_SECONDS;
}

void BH1750_Tick(float dt)
{
    uint8_t bytes[2];
    if (dt <= 0.0f) return;
    if (bh1750.age_seconds < 60.0f) bh1750.age_seconds += dt;
    if (bh1750.age_seconds > BH1750_MAX_AGE_SECONDS) bh1750.valid = 0;
    if (retry_seconds > 0.0f) {
        retry_seconds -= dt;
        return;
    }
    if (init_index < sizeof(init_commands)) {
        if (IIC_Send_Command(BH1750_ADDRESS_7BIT << 1,
                             init_commands[init_index]) != 0) {
            bh1750_fail();
            return;
        }
        ++init_index;
        wait_seconds = 0.0f;
        return;
    }
    wait_seconds += dt;
    if (wait_seconds + 0.00001f < BH1750_SAMPLE_SECONDS) return;
    wait_seconds = 0.0f;
    if (IIC_Receive_Bytes(BH1750_ADDRESS_7BIT << 1, bytes, 2) != 0) {
        bh1750_fail();
        return;
    }
    bh1750.raw = ((uint16_t)bytes[0] << 8) | bytes[1];
    bh1750.lux = (float)bh1750.raw / 1.2f;
    bh1750.saturated = bh1750.raw == 65535U;
    bh1750.valid = !bh1750.saturated;
    bh1750.age_seconds = 0.0f;
    ++bh1750.sample_id;
}
