#include "ALL_DEFINE.h"
#include "light_link.h"
#include "light_task.h"
#include "bh1750.h"
#include "light_link_protocol.h"
#include "auto_flight.h"
#include "flow.h"

void LightLink_OnControlFrame(const uint8_t *frame)
{
    LightLinkStatus status;
    uint8_t packet[32];
    uint32_t mask;
    if (!LightLink_Requested(frame)) return;
    mask = __get_PRIMASK();
    __disable_irq();
    status.event_id = light_task.event_id;
    status.alert_elapsed_ms = LightTask_AlertElapsedMs();
    status.state = (uint8_t)light_task.state;
    status.flags = (LightTask_AlertActive() ? LIGHT_LINK_ALERT : 0U) |
        (LightTask_RemoteLocked() ? LIGHT_LINK_LOCKED : 0U) |
        (light_task.source_detected ? LIGHT_LINK_DETECTED : 0U) |
        (light_task.state == LIGHT_COMPLETE ? LIGHT_LINK_COMPLETE : 0U) |
        (light_task.sensor_fault ? LIGHT_LINK_SENSOR_BAD : 0U);
    status.lux = (uint16_t)bh1750.lux;
    status.ambient_lux = (uint16_t)light_task.ambient_lux;
    status.arrival_lux = (uint16_t)light_task.arrival_lux;
    status.diagnostic_valid = 1;
    status.auto_state = (uint8_t)auto_flight.state;
    status.auto_error = (uint8_t)auto_flight.error;
    status.start_request_id = auto_flight.start_request_id;
    status.range_raw = flow_raw_height;
    status.height_mm = (uint16_t)auto_flight.height_mm;
    status.sensor_flags = (flow_height_valid ? 1U : 0U) |
        (bh1750.valid ? 2U : 0U) | (flow_data.squal > 30U ? 4U : 0U);
    status.aux3 = Remote.AUX3;
    __set_PRIMASK(mask);
    LightLink_Encode(packet, &status);
    /* Foreground only; interrupts and the motor scheduler keep running. */
    NRF24L01_SendReply(packet);
}
