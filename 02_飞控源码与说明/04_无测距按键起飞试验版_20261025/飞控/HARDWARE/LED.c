/*******************************************************************
 *@title LED system
 *@brief flight light
 *@brief 
 *@time  2016.10.19
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 ******************************************************************/
#include "stm32f10x.h"
#include "LED.h"
#include "ALL_DATA.h"
#include "auto_flight.h"
#include "light_task.h"





////右前灯			 
#define fLED1_H()  TIM3->CCR1=1000 //暗
#define fLED1_L()  TIM3->CCR1=500  //亮
#define fLED1_Toggle()  TIM3->CCR1^=(1000^500)//闪烁
////左前灯			 
#define fLED2_H()  TIM3->CCR2=1000 //暗
#define fLED2_L()  TIM3->CCR2=500  //亮
#define fLED2_Toggle()  TIM3->CCR2^=(1000^500)//闪烁
////左后灯			 
#define bLED3_H()  TIM3->CCR3=1000 //暗
#define bLED3_L()  TIM3->CCR3=500  //亮
#define bLED3_Toggle()  TIM3->CCR3^=(1000^500)//闪烁
//-------------------------------------------------
////左后灯			 
#define bLED4_H()  TIM3->CCR4=1000 //暗
#define bLED4_L()  TIM3->CCR4=500  //亮
#define bLED4_Toggle()  TIM3->CCR4^=(1000^500)//闪烁
//-------------------------------------------------

//-------------------------------------------------
//---------------------------------------------------------
/*     you can select the LED statue on enum contains            */
sLED LED = {300,AllFlashLight};  //LED initial statue is off;
                             //default 300ms flash the status

/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
/* Alarm indications take priority over the legacy mode lights. */
static void auto_alarm_leds(uint8_t mask)
{
    uint16_t front = (mask & AUTO_BLUE_LED_MASK) ? 500U : 1000U;
    uint16_t rear = (mask & AUTO_RED_LED_MASK) ? 500U : 1000U;
    TIM3->CCR1 = TIM3->CCR2 = front;
    TIM3->CCR3 = TIM3->CCR4 = rear;
}
void PilotLED() //flash 300MS interval
{
    uint8_t phase = (uint8_t)((SysTick_count / 67U) & 1U);
    static uint8_t alarm_was_active;
    if (auto_flight.alarm != AUTO_ALARM_NONE) {
        uint8_t mask;
        if (auto_flight.alarm == AUTO_ALARM_LOW)
            mask = phase ? AUTO_RED_LED_MASK : 0U;
        else if (auto_flight.alarm == AUTO_ALARM_HIGH)
            mask = phase ? AUTO_BLUE_LED_MASK : 0U;
        else mask = phase ? AUTO_RED_LED_MASK : AUTO_BLUE_LED_MASK;
        auto_alarm_leds(mask);
        alarm_was_active = 1;
        return;
    }
    if (LightTask_AlertActive()) {
        auto_alarm_leds(LightTask_AlertLedOn() ?
                        (AUTO_BLUE_LED_MASK | AUTO_RED_LED_MASK) : 0U);
        alarm_was_active = 1;
        return;
    }
    if (alarm_was_active) {
        auto_alarm_leds(0U);
        alarm_was_active = 0;
    }
	static uint32_t LastTime = 0;
//	static u8 last_LED_status
	if((uint32_t)(SysTick_count - LastTime) * 3U < LED.FlashTime)
	{

		return;
	}
	else
		LastTime = SysTick_count;
	if(!ALL_flag.unlock)//如果锁定状态无论如何4个LED都是闪烁
	{
		LED.status = AllFlashLight;	
	}
	else
	{
		if(!ALL_flag.flow_control&&!ALL_flag.height_lock)//如果不是定高也不是定点状态，则为4个LED常亮
		{
			LED.status = AlwaysOn;	
		}
		else if(ALL_flag.flow_control)//定点状态，4个灯全暗
		{
			LED.status = AlwaysOff;	
		}
		else
		{
			LED.status = WARNING;	//定高状态，机头两个灯闪烁	
		}
	}
	
	
	switch(LED.status)
	{
		case AlwaysOff:      //常暗   
			fLED1_H();
			fLED2_H();
			bLED3_H();
			bLED4_H();
			break;
		case AllFlashLight:				  //全部同时闪烁		
			fLED1_Toggle();
			fLED2_Toggle();	
			bLED3_Toggle();  
			bLED4_Toggle();  
	
		  break;
		case AlwaysOn:  //常亮
			fLED1_L();
			fLED2_L();
			bLED3_L();
			bLED4_L();

		  break;
		case WARNING://前面两灯闪烁
			fLED1_Toggle();		
			fLED2_Toggle();	
			bLED3_H();
			bLED4_H();
			break;
		default:
			LED.status = AlwaysOff;
			break;
	}
}

/**************************END OF FILE*********************************/



