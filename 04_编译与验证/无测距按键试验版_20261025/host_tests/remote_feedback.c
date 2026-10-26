
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
static int beep_pin;
volatile uint32_t SysTick_count;
#define BEEP_H beep_pin=1
#define BEEP_L beep_pin=0
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t m) {}
static uint8_t NRF24L01_RxPacket(uint8_t *p) { return 1; }
static void RC_Analy(void) {}
#ifndef LIGHT_LINK_PROTOCOL_H
#define LIGHT_LINK_PROTOCOL_H



/* Shared by the aircraft and transmitter. Existing eight RC channels and the
 * 32-byte additive checksum remain unchanged. Feedback is opt-in. */
#define LIGHT_LINK_ALERT       0x01U
#define LIGHT_LINK_LOCKED      0x02U
#define LIGHT_LINK_DETECTED    0x04U
#define LIGHT_LINK_COMPLETE    0x08U
#define LIGHT_LINK_SENSOR_BAD  0x10U

typedef struct {
    uint32_t event_id;
    uint16_t alert_elapsed_ms, lux, ambient_lux, arrival_lux;
    uint8_t state, flags;
    /* Optional diagnostic extension in previously unused payload bytes. */
    uint8_t diagnostic_valid, auto_state, auto_error, sensor_flags;
    uint16_t start_request_id, range_raw, height_mm, aux3;
} LightLinkStatus;

static __inline uint8_t LightLink_Checksum(const uint8_t *data)
{
    uint8_t sum = 0, i;
    for (i = 0; i < 31U; ++i) sum = (uint8_t)(sum + data[i]);
    return sum;
}
static __inline uint8_t LightLink_Requested(const uint8_t *data)
{
    return data[20] == 'L' && data[21] == 'T' &&
        data[22] == 1U && data[23] == 0xA5U;
}
static __inline void LightLink_Put16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value >> 8); out[1] = (uint8_t)value;
}
static __inline uint16_t LightLink_Get16(const uint8_t *in)
{
    return ((uint16_t)in[0] << 8) | in[1];
}
static __inline void LightLink_Encode(uint8_t *out, const LightLinkStatus *s)
{
    memset(out, 0, 32);
    out[0] = 0xAA; out[1] = 0xB0; out[2] = 1; out[3] = 27;
    out[4] = (uint8_t)(s->event_id >> 24);
    out[5] = (uint8_t)(s->event_id >> 16);
    out[6] = (uint8_t)(s->event_id >> 8);
    out[7] = (uint8_t)s->event_id;
    LightLink_Put16(out + 8, s->alert_elapsed_ms);
    out[10] = s->state; out[11] = s->flags;
    LightLink_Put16(out + 12, s->lux);
    LightLink_Put16(out + 14, s->ambient_lux);
    LightLink_Put16(out + 16, s->arrival_lux);
    if (s->diagnostic_valid) {
        out[18] = 0xD1;
        out[19] = s->auto_state; out[20] = s->auto_error;
        LightLink_Put16(out + 22, s->start_request_id);
        LightLink_Put16(out + 24, s->range_raw);
        LightLink_Put16(out + 26, s->height_mm);
        out[28] = s->sensor_flags;
        LightLink_Put16(out + 29, s->aux3);
    }
    out[31] = LightLink_Checksum(out);
}
static __inline uint8_t LightLink_Decode(const uint8_t *in, LightLinkStatus *s)
{
    if (in[0] != 0xAA || in[1] != 0xB0 || in[2] != 1 || in[3] != 27 ||
        in[31] != LightLink_Checksum(in) || in[10] > 6U ||
        (in[11] & 0xE0U) || LightLink_Get16(in + 8) > 3000U) return 0;
    s->event_id = ((uint32_t)in[4] << 24) | ((uint32_t)in[5] << 16) |
        ((uint32_t)in[6] << 8) | in[7];
    s->alert_elapsed_ms = LightLink_Get16(in + 8);
    s->state = in[10]; s->flags = in[11];
    s->lux = LightLink_Get16(in + 12);
    s->ambient_lux = LightLink_Get16(in + 14);
    s->arrival_lux = LightLink_Get16(in + 16);
    s->diagnostic_valid = in[18] == 0xD1;
    s->auto_state = s->diagnostic_valid ? in[19] : 0;
    s->auto_error = s->diagnostic_valid ? in[20] : 0;
    s->start_request_id = s->diagnostic_valid ? LightLink_Get16(in + 22) : 0;
    s->range_raw = s->diagnostic_valid ? LightLink_Get16(in + 24) : 0;
    s->height_mm = s->diagnostic_valid ? LightLink_Get16(in + 26) : 0;
    s->sensor_flags = s->diagnostic_valid ? in[28] : 0;
    s->aux3 = s->diagnostic_valid ? LightLink_Get16(in + 29) : 0;
    if (s->diagnostic_valid && (s->auto_state > 5U || s->auto_error > 10U ||
        (s->sensor_flags & 0xF8U))) return 0;
    return 1;
}
#endif
#ifndef LIGHT_FEEDBACK_H
#define LIGHT_FEEDBACK_H



extern volatile LightLinkStatus remote_light_status;
void RemoteFeedback_Init(void);
uint8_t RemoteFeedback_Accept(const uint8_t *packet, uint32_t now_ms);
void RemoteFeedback_KeyClick(uint32_t now_ms);
void RemoteFeedback_BuzzerTick(uint32_t now_ms);
void RemoteControl_Poll(void);
#endif






volatile LightLinkStatus remote_light_status;
static volatile uint32_t alert_start_ms, click_start_ms;
static volatile uint8_t alert_seen, alert_active, click_active;
static uint32_t last_send_ms;
static uint8_t control_ready;
static volatile uint32_t reject_start_ms;
static volatile uint8_t reject_active, reject_beeps;
static uint16_t last_request_id;
static volatile uint8_t open_takeover_wait;
static volatile uint32_t open_feedback_ms;

void RemoteFeedback_Init(void)
{
    memset((void *)&remote_light_status, 0, sizeof(remote_light_status));
    alert_seen = alert_active = click_active = control_ready = 0;
    reject_active = reject_beeps = 0;
    last_request_id = 0;
    open_takeover_wait = 0;
    open_feedback_ms = 0;
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
    /* This independent flight version uses state 3 for throttle matching.
     * A short chirp every 700 ms ends after manual takeover or link timeout. */
    open_takeover_wait = s.diagnostic_valid && s.auto_state == 3U;
    open_feedback_ms = now_ms;
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
    uint8_t waiting = open_takeover_wait && (uint32_t)(now_ms-open_feedback_ms)<500U;
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
    if (!alert_active && !reject_active && waiting)
        on = (now_ms % 700U) < 80U;
    if (!alert_active && !reject_active && !waiting && click_active) {
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

int main(int argc,char **argv) {
    int i,prev=0,rises=0; uint8_t p[32]; LightLinkStatus s={0};
    RemoteFeedback_Init(); s.diagnostic_valid=1; s.auto_state=3;
    LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,0));
    if(!strcmp(argv[1],"takeover_wait_notice")) {
        for(i=0;i<3000;++i) {
            if(i%20==0) assert(RemoteFeedback_Accept(p,i));
            RemoteFeedback_BuzzerTick(i);
            if(beep_pin && !prev)++rises; prev=beep_pin;
        }
        assert(rises==5);
    } else if(!strcmp(argv[1],"notice_expiry_keeps_key_click")) {
        RemoteFeedback_BuzzerTick(600); assert(!beep_pin);
        RemoteFeedback_KeyClick(600); RemoteFeedback_BuzzerTick(600); assert(beep_pin);
        RemoteFeedback_BuzzerTick(631); assert(!beep_pin);
    } else if(!strcmp(argv[1],"manual_feedback_ends_notice")) {
        s.auto_state=4; LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,100));
        RemoteFeedback_BuzzerTick(700); assert(!beep_pin);
    } else if(!strcmp(argv[1],"stop_feedback_ends_notice")) {
        s.auto_state=0; LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,100));
        RemoteFeedback_BuzzerTick(700); assert(!beep_pin);
    } else if(!strcmp(argv[1],"rejection_notice")) {
        s.auto_state=0; s.auto_error=2; s.start_request_id=1;
        LightLink_Encode(p,&s);
        for(i=0;i<1500;++i) {
            if(i%20==0) assert(RemoteFeedback_Accept(p,i));
            RemoteFeedback_BuzzerTick(i);
            if(beep_pin && !prev)++rises; prev=beep_pin;
        }
        assert(rises==2);
    } else if(!strcmp(argv[1],"malformed_feedback")) {
        p[31]++; assert(!RemoteFeedback_Accept(p,0));
    } else assert(0);
    puts("passed"); return 0;
}
