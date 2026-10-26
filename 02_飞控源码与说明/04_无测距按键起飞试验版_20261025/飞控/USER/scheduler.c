#include "ALL_DEFINE.h"
#include "auto_flight.h"
#include "bh1750.h"
#include "light_task.h"
void TIM1_UP_IRQHandler(void)
{
    static uint8_t count_6ms;
    if (TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET) {
        ++SysTick_count;
        MpuGetData();
        if (++count_6ms>=2U) {
            count_6ms=0;
            GetAngle(&MPU6050,&Angle,0.006f);
            BH1750_Tick(0.006f);
            AutoFlight_Tick(0.006f);
            LightTask_Tick(0.006f);
            PilotLED();
        }
        FlightPidControl(0.003f);
        MotorControl();
        TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
    }
}
