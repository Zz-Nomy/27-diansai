#include "sys.h"
#include "remote.h"
#include "nrf24l01.h"
#include "light_feedback.h"
#include <string.h>

volatile LightLinkStatus remote_light_status;
static volatile uint32_t alert_start_ms, click_start_ms;
static volatile uint8_t alert_seen, alert_active, click_active;
static uint32_t last_send_ms;
static uint8_t control_ready;
static volatile uint32_t reject_start_ms;
static volatile uint8_t reject_active, reject_beeps;
static uint16_t last_request_id;

void RemoteFeedback_Init(void)
{
    memset((void *)&remote_light_status, 0, sizeof(remote_light_status));
    alert_seen = alert_active = click_active = control_ready = 0;
    reject_active = reject_beeps = 0;
    last_request_id = 0;
    last_send_ms = SysTick_count;
}
uint8_t RemoteFeedback_Accept(const uint8_t *packet, uint32_t now_ms)
{
    LightLinkStatus s;
    uint32_t mask;
    if (!LightLink_Decode(packet, &s)) return 0;
    mask = __get_PRIMASK();
    __disable_irq();
    if (!s.event_id && !(s.flags & LIGHT_LINK_ALERT)) alert_seen = 0;
    if ((s.flags & LIGHT_LINK_ALERT) && s.event_id &&
        (!alert_seen || s.event_id != remote_light_status.event_id)) {
        alert_start_ms = now_ms - s.alert_elapsed_ms;
        alert_seen = alert_active = 1;
    }
    if (!(s.flags & LIGHT_LINK_ALERT)) alert_active = 0;
    if (s.diagnostic_valid && s.start_request_id != last_request_id) {
        last_request_id = s.start_request_id;
        /* One rejection notification per AUX3 attempt; repeats never extend it.
         * Runtime flight faults are not counted as a rejected takeoff. */
        if (s.auto_state == 0U && s.auto_error >= 1U && s.auto_error <= 5U) {
            reject_start_ms = now_ms;
            reject_beeps = s.auto_error;
            reject_active = 1;
        } else reject_active = 0;
    }
    remote_light_status = s;
    __set_PRIMASK(mask);
    return 1;
}
void RemoteFeedback_KeyClick(uint32_t now_ms)
{
    click_start_ms = now_ms;
    click_active = 1;
}
void RemoteFeedback_BuzzerTick(uint32_t now_ms)
{
    uint32_t elapsed;
    uint8_t on = 0;
    if (alert_active) {
        elapsed = (uint32_t)(now_ms - alert_start_ms);
        if (elapsed >= 3000U) alert_active = 0;
        else on = (elapsed % 1000U) < 150U;
    }
    if (!alert_active && reject_active) {
        elapsed = (uint32_t)(now_ms - reject_start_ms);
        if (elapsed >= (uint32_t)reject_beeps * 350U) reject_active = 0;
        else on = (elapsed % 350U) < 120U;
    }
    if (!alert_active && !reject_active && click_active) {
        if ((uint32_t)(now_ms - click_start_ms) < 30U) on = 1;
        else click_active = 0;
    }
    if (on) { BEEP_H; } else { BEEP_L; }
}
/* 50 Hz RC uplink. Between frames the radio listens for aircraft feedback.
 * No ACK-payload extension or additional radio module is required. */
void RemoteControl_Poll(void)
{
    uint8_t packet[32];
    uint32_t now = SysTick_count;
    if (!control_ready) { control_ready = 1; last_send_ms = now - 20U; }
    if (NRF24L01_RxPacket(packet) == 0) RemoteFeedback_Accept(packet, now);
    if ((uint32_t)(now - last_send_ms) >= 20U) {
        last_send_ms = now;
        RC_Analy();
    }
}
