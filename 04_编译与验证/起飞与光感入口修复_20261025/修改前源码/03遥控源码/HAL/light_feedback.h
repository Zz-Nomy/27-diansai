#ifndef LIGHT_FEEDBACK_H
#define LIGHT_FEEDBACK_H
#include "stm32f10x.h"
#include "light_link_protocol.h"

extern volatile LightLinkStatus remote_light_status;
void RemoteFeedback_Init(void);
uint8_t RemoteFeedback_Accept(const uint8_t *packet, uint32_t now_ms);
void RemoteFeedback_KeyClick(uint32_t now_ms);
void RemoteFeedback_BuzzerTick(uint32_t now_ms);
void RemoteControl_Poll(void);
#endif
