
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define assert(c) do { if(!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(2); } } while(0)

volatile uint32_t SysTick_count;
uint32_t SystemCoreClock=72000000U;
static struct { uint32_t LOAD,VAL; } tick;
#define SysTick (&tick)
static void SysTick_Config(uint32_t n) { tick.LOAD=n-1; tick.VAL=n; }
static int laser_calls, light_calls;
static float light_seconds;
static int mpu,angle;
#define MPU6050 mpu
#define Angle angle
#define TIM1 1
#define TIM_IT_Update 1
#define RESET 0
static int TIM_GetITStatus(int t,int i) { return 1; }
static void TIM_ClearITPendingBit(int t,int i) {}
static void MpuGetData(void) {}
static void GetAngle(int *a,int *b,float dt) {}
static void flow_get_data(void) { ++laser_calls; }
static void BH1750_Tick(float dt) {}
static void AutoFlight_Tick(float dt) {}
static void LightTask_Tick(float dt) { ++light_calls; light_seconds+=dt; }
static void HeightPidControl(float dt) {}
static void PilotLED(void) {}
static void flow_data_sins(void) {}
static void FlowPidControl(float dt) {}
static void FlightPidControl(float dt) {}
static void MotorControl(void) {}
/*******************************************************************
 *MPU6050
 *@brief 
 *@brief 
 *@time  2016.1.8
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 *非授权使用人员，禁止使用。禁止传阅，违者一经发现，侵权处理。
 ******************************************************************/




/* Only TIM1 updates SysTick_count (one tick = 3 ms). */
static volatile uint32_t systick_ms = 0;


/**
  * @brief  This function handles SysTick_Handler exception.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	++systick_ms;
}



void cycleCounterInit(void)
{
	//滴答定时器初始化
	SysTick_Config(SystemCoreClock / 1000);	
}

/***********************************************************************
 * 获得系统当前运行的us值 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
uint32_t GetSysTime_us(void) //获取当前系统的运行时间 单位us
{
    uint32_t ms, value, reload = SysTick->LOAD + 1U;
    do { ms = systick_ms; value = SysTick->VAL; } while (ms != systick_ms);
    return ms * 1000U + (reload - value) / (SystemCoreClock / 1000000U);
}

//    毫秒级延时函数	 
void delay_ms(uint16_t nms)
{
	uint32_t start_ms = systick_ms;
	while ((uint32_t)(systick_ms - start_ms) < nms);	  	  
}


void delay_us(unsigned int i)
 {  
	char x=0;   
    while( i--)
    {	
       for(x=1;x>0;x--);
    }
 }		  







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
            BH1750_Tick(0.006f);
            AutoFlight_Tick(0.006f);
            LightTask_Tick(0.006f);
            HeightPidControl(0.006f);
            PilotLED();
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

int main(int argc,char **argv) {
    int ms; cycleCounterInit();
    for(ms=1;ms<=3000;++ms) { SysTick_Handler(); if(ms%3==0) TIM1_UP_IRQHandler(); }
    assert(SysTick_count==1000 && laser_calls==500 && light_calls==500);
    assert(fabsf(light_seconds-3.0f)<0.0001f);
    assert(GetSysTime_us()==3000000U);
    puts("passed");return 0;
}
