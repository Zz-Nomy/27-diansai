#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "control.h"
#include "myMath.h"
#include "LED.h"
#include "remote.h"
#include "auto_flight.h"
#include "light_link.h"
#include "imu.h"
uint8_t RC_rxData[32];
uint16_t last_Remote_AUX1=1000,last_Remote_AUX2=1000;
uint16_t last_Remote_AUX3=1000,last_Remote_AUX4=1000;
int16_t roll_offset,pitch_offset;
void remote_unlock(void);
void Remote_ArmForAutoFlight(void)
{
    roll_offset=(int16_t)Remote.roll-1500;
    pitch_offset=(int16_t)Remote.pitch-1500;
    pidRoll.offset=-roll_offset*0.04f;
    pidPitch.offset=-pitch_offset*0.04f;
    pidRoll.desired=pidPitch.desired=0.0f;
    imu_rest();
}
void remote_unlock(void)
{
    /* Stick emergency remains available; stick arming is deliberately absent.
     * Only a fresh AUX3 press after AUX2 can start this experimental version. */
    if (Remote.thr<1050U && Remote.yaw<1150U)
        AutoFlight_EmergencyStop(AUTO_OK);
}
void RC_Analy(void)
{
    static uint32_t last_reinit_tick;
    uint8_t i,sum=0;
    uint32_t mask;
    uint16_t channel[8];
    if (NRF24L01_RxPacket(RC_rxData)!=0) return;
    for (i=0;i<31U;++i) sum=(uint8_t)(sum+RC_rxData[i]);
    if (RC_rxData[31]!=sum || RC_rxData[0]!=0xAA || RC_rxData[1]!=0xAF ||
        RC_rxData[2]!=0x03 || RC_rxData[3]!=27U) return;
    for (i=0;i<8U;++i) {
        channel[i]=((uint16_t)RC_rxData[4U+2U*i]<<8)|RC_rxData[5U+2U*i];
        if (channel[i]<900U || channel[i]>2100U) return;
        channel[i]=LIMIT(channel[i],1000U,2000U);
    }
    mask=__get_PRIMASK(); __disable_irq();
    Remote.roll=channel[0]; Remote.pitch=channel[1];
    Remote.thr=channel[2]; Remote.yaw=channel[3];
    Remote.AUX1=channel[4]; Remote.AUX2=channel[5];
    Remote.AUX3=channel[6]; Remote.AUX4=channel[7];
    AutoFlight_OnRcFrame();
    __set_PRIMASK(mask);
    if (auto_flight.state==AUTO_MANUAL && ALL_flag.unlock) {
        pidPitch.desired=(-(float)Remote.pitch+1500.0f+pitch_offset)*0.04f;
        pidRoll.desired=(-(float)Remote.roll+1500.0f+roll_offset)*0.04f;
        if (Remote.yaw>1820U) pidYaw.desired-=0.5f;
        else if (Remote.yaw<1180U) pidYaw.desired+=0.5f;
    }
    remote_unlock();
    LightLink_OnControlFrame(RC_rxData);
    if (auto_flight.state==AUTO_RC_FAULT &&
        (uint32_t)(SysTick_count-last_reinit_tick)>OPEN_RC_LOSS_TICKS) {
        last_reinit_tick=SysTick_count;
        NRF24L01_init();
    }
}
