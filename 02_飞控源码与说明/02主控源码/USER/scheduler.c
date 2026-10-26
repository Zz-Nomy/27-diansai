#include "ALL_DEFINE.h"
#include "scheduler.h"
#include "flow.h"
#include "auto_flight.h"

void TIM1_UP_IRQHandler(void)
{
    static uint8_t count_6ms, count_24ms;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        ++SysTick_count; /* One tick is 3 ms, not the CPU SysTick's 1 ms. */
        ++count_6ms;
        ++count_24ms;
        MpuGetData();
        if (count_6ms >= 2U) {
            count_6ms = 0;
            GetAngle(&MPU6050, &Angle, 0.006f);
            flow_get_data(); /* Acquire before estimating/controlling height. */
            AutoFlight_Tick(0.006f);
            HeightPidControl(0.006f);
        }
        if (count_24ms >= 8U) {
            count_24ms = 0;
            flow_data_sins();
            FlowPidControl(0.024f);
        }
        FlightPidControl(0.003f);
        MotorControl();
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
