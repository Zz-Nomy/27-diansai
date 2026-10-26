/*******************************************************************
 *MPU6050
 *@brief 
 *@brief 
 *@time  2016.1.8
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 *非授权使用人员，禁止使用。禁止传阅，违者一经发现，侵权处理。
 ******************************************************************/
#include "stm32f10x.h"
#include "misc.h"
#include "delay.h"
#include "ALL_DATA.h"
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
