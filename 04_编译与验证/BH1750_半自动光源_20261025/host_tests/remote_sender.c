
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

static struct { uint16_t roll,pitch,thr,yaw,AUX1,AUX2,AUX3,AUX4; } Remote;
static uint8_t tx_data[32]={0xAA,0xAF,3,27}, sent_packet[32];
static int tx_mode, listening;
static void TX_Mode(void) { ++tx_mode; }
static uint8_t NRF24L01_TxPacket(uint8_t *p) { memcpy(sent_packet,p,32); return 0; }
static void NRF24L01_Listen(void) { ++listening; }
void NRF_SEND(void)
{
    uint8_t i;
    uint16_t channels[8];
    channels[0] = Remote.roll; channels[1] = Remote.pitch;
    channels[2] = Remote.thr; channels[3] = Remote.yaw;
    channels[4] = Remote.AUX1; channels[5] = Remote.AUX2;
    channels[6] = Remote.AUX3; channels[7] = Remote.AUX4;
    memset(tx_data + 4, 0, 28);
    for (i = 0; i < 8U; ++i) {
        tx_data[4U + i * 2U] = (uint8_t)(channels[i] >> 8);
        tx_data[5U + i * 2U] = (uint8_t)channels[i];
    }
    tx_data[20] = 'L'; tx_data[21] = 'T';
    tx_data[22] = 1; tx_data[23] = 0xA5;
    tx_data[31] = LightLink_Checksum(tx_data);
    TX_Mode();
    NRF24L01_TxPacket(tx_data);
    NRF24L01_Listen();
}
int main(int argc,char **argv) {
    uint16_t c[8]={1500,1600,1000,1400,1000,2000,2000,1000};
    int i; memcpy(&Remote,c,sizeof c); NRF_SEND();
    for(i=0;i<8;++i) assert(LightLink_Get16(sent_packet+4+2*i)==c[i]);
    assert(LightLink_Requested(sent_packet) && sent_packet[31]==LightLink_Checksum(sent_packet));
    for(i=24;i<31;++i) assert(sent_packet[i]==0);
    assert(tx_mode==1 && listening==1); puts("passed");return 0;
}
