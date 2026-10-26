
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define assert(c) do { if(!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(2); } } while(0)
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

static int beep_pin, send_count;
volatile uint32_t SysTick_count;
#define BEEP_H beep_pin=1
#define BEEP_L beep_pin=0
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t value) {}
static uint8_t NRF24L01_RxPacket(uint8_t *out) { return 1; }
static void RC_Analy(void) { ++send_count; }
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

int main(int argc,char **argv) {
    uint8_t p[32]; int i,rises=0,prev=0,total=0;
    LightLinkStatus s={0};
    RemoteFeedback_Init(); s.event_id=1; s.flags=LIGHT_LINK_ALERT; s.state=3;
    LightLink_Encode(p,&s);
    if(!strcmp(argv[1],"beep_three_seconds")) {
        for(i=0;i<4000;++i) {
            if(i%20==0) { s.alert_elapsed_ms=i<3000?i:3000; if(i>=3000)s.flags=0; LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,i)); }
            RemoteFeedback_BuzzerTick(i);
            if(beep_pin && !prev)++rises; prev=beep_pin; total+=beep_pin;
        }
        assert(rises==3 && total==450 && !beep_pin);
    } else if(!strcmp(argv[1],"duplicate_no_extension")) {
        for(i=0;i<4000;++i) {
            if(i%20==0) assert(RemoteFeedback_Accept(p,i));
            RemoteFeedback_BuzzerTick(i);
            if(beep_pin && !prev)++rises; prev=beep_pin;
        }
        assert(rises==3 && !beep_pin);
    } else if(!strcmp(argv[1],"bad_feedback")) {
        p[31]++; assert(!RemoteFeedback_Accept(p,0));
        RemoteFeedback_BuzzerTick(1); assert(!beep_pin);
    } else if(!strcmp(argv[1],"late_feedback")) {
        s.alert_elapsed_ms=1100; LightLink_Encode(p,&s);
        assert(RemoteFeedback_Accept(p,5000));
        RemoteFeedback_BuzzerTick(5000); assert(beep_pin);
        RemoteFeedback_BuzzerTick(5050); assert(!beep_pin);
        RemoteFeedback_BuzzerTick(5900); assert(beep_pin);
        RemoteFeedback_BuzzerTick(6900); assert(!beep_pin);
    } else if(!strcmp(argv[1],"key_click_priority")) {
        assert(RemoteFeedback_Accept(p,0)); RemoteFeedback_KeyClick(500);
        RemoteFeedback_BuzzerTick(500); assert(!beep_pin);
        RemoteFeedback_KeyClick(3100); RemoteFeedback_BuzzerTick(3100); assert(beep_pin);
        RemoteFeedback_BuzzerTick(3130); assert(!beep_pin);
    } else if(!strcmp(argv[1],"remote_poll_period")) {
        for(i=0;i<1000;++i) { SysTick_count=i; RemoteControl_Poll(); }
        assert(send_count==50);
    } else if(!strcmp(argv[1],"clock_wrap")) {
        assert(RemoteFeedback_Accept(p,0xFFFFFF00U));
        RemoteFeedback_BuzzerTick(0xFFFFFF00U); assert(beep_pin);
        RemoteFeedback_BuzzerTick(0x00000AB8U); assert(!beep_pin);
    } else if(!strcmp(argv[1],"takeoff_reject_beeps")) {
        s.event_id=0; s.flags=0; s.state=0;
        s.diagnostic_valid=1; s.auto_error=3; s.start_request_id=1;
        s.range_raw=0; s.height_mm=60; s.aux3=2000;
        LightLink_Encode(p,&s);
        for(i=0;i<2000;++i) {
            if(i%20==0) assert(RemoteFeedback_Accept(p,i));
            RemoteFeedback_BuzzerTick(i);
            if(beep_pin && !prev)++rises; prev=beep_pin; total+=beep_pin;
        }
        assert(rises==3 && total==360 && !beep_pin);
        assert(remote_light_status.auto_error==3 && remote_light_status.aux3==2000);
    } else if(!strcmp(argv[1],"takeoff_success_no_reject_beep")) {
        s.flags=0; s.diagnostic_valid=1; s.start_request_id=1; s.auto_state=1;
        LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,0));
        RemoteFeedback_BuzzerTick(1); assert(!beep_pin);
    } else if(!strcmp(argv[1],"bad_diagnostic")) {
        s.flags=0; s.diagnostic_valid=1; s.auto_error=255;
        LightLink_Encode(p,&s); assert(!RemoteFeedback_Accept(p,0));
        RemoteFeedback_BuzzerTick(1); assert(!beep_pin);
    } else if(!strcmp(argv[1],"legacy_feedback_diagnostic_reset")) {
        s.flags=0; s.diagnostic_valid=1; s.start_request_id=1; s.auto_state=1;
        LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,0));
        s.diagnostic_valid=0; LightLink_Encode(p,&s);
        assert(RemoteFeedback_Accept(p,20));
        assert(!remote_light_status.diagnostic_valid && !remote_light_status.start_request_id);
    } else assert(0);
    puts("passed");return 0;
}
