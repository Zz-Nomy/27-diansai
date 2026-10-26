/*******************************************************************
 *@title Remote Control;
 *@brief remote data process on polling every 10ms.
 *@brief remote control on interrupt every 24ms
 *@time  2016.11.21
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 ******************************************************************/
#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "mpu6050.h"
#include "auto_flight.h"
#include "imu.h"

#define SUCCESS 0
#undef FAILED
#define FAILED  1
/*****************************************************************************************
 *  通道数据处理
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
	uint8_t RC_rxData[32];
void remote_unlock(void);
uint16_t last_Remote_AUX1 = 1000;
uint16_t last_Remote_AUX3 = 1000;
uint16_t last_Remote_AUX4 = 1000;
uint16_t last_Remote_AUX2 = 1000;

int16_t roll_offset; //微调值
int16_t pitch_offset;
 

void RC_Analy(void)  
{
		static uint32_t last_valid_tick, last_reinit_tick;
/*             Receive  and check RC data                               */	
	if(NRF24L01_RxPacket(RC_rxData)==SUCCESS)
	{ 	

		uint8_t i;
		uint8_t CheckSum=0;
		for(i=0;i<31;i++)
		{
			CheckSum +=  RC_rxData[i];
		}
		if(RC_rxData[31]==CheckSum && RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)  //如果接收到的遥控数据正确
        {
            /* Reject malformed channels before publishing a valid RC frame. */
            for (i = 4; i < 20; i += 2) {
                uint16_t value = ((uint16_t)RC_rxData[i] << 8) | RC_rxData[i+1];
                if (value < 900U || value > 2100U) return;
            }
            _st_Remote next_remote = Remote;
            uint32_t previous_mask;
            last_valid_tick = SysTick_count;
			  next_remote.roll = ((uint16_t)RC_rxData[4]<<8) | RC_rxData[5];  //通道1
				next_remote.roll = LIMIT(next_remote.roll,1000,2000);
				next_remote.pitch = ((uint16_t)RC_rxData[6]<<8) | RC_rxData[7];  //通道2
				next_remote.pitch = LIMIT(next_remote.pitch,1000,2000);
				next_remote.thr = 	((uint16_t)RC_rxData[8]<<8) | RC_rxData[9];   //通道3
				next_remote.thr = 	LIMIT(next_remote.thr,1000,2000);
				next_remote.yaw =  ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];   //通道4
				next_remote.yaw =  LIMIT(next_remote.yaw,1000,2000);
				next_remote.AUX1 =  ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];   //通道5  左上角按键都属于通道5  
				next_remote.AUX1 =  LIMIT(next_remote.AUX1,1000,2000);
				next_remote.AUX2 =  ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];   //通道6  右上角按键都属于通道6 
				next_remote.AUX2 =  LIMIT(next_remote.AUX2,1000,2000);
				next_remote.AUX3 =  ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];   //通道7  左下边按键都属于通道7 
				next_remote.AUX3 =  LIMIT(next_remote.AUX3,1000,2000);
				next_remote.AUX4 =  ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];   //通道8  右下边按键都属于通道6  
				next_remote.AUX4 = LIMIT(next_remote.AUX4,1000,2000);
				/* Publish the complete RC sample as one short critical section. */
                previous_mask = __get_PRIMASK();
                __disable_irq();
                Remote = next_remote;
                AutoFlight_OnRcFrame();
                __set_PRIMASK(previous_mask);		

							//通道5作为定高控制 每按一下就会在1000和2000之间切换 
							if(Remote.AUX1!=last_Remote_AUX1 && !AutoFlight_Active()) //如果大于1700 则进入定高模式
							{
								last_Remote_AUX1=Remote.AUX1;
								ALL_flag.height_lock = ALL_flag.flow_control ^= 1;//取反
			
							}														
				{
						
							if(!ALL_flag.flow_control && !AutoFlight_Active())//如果不处于定点模式则飞行角度由遥控的遥杆控制
							{
										 float roll_pitch_ratio = 0.04f;

										pidPitch.desired =(-(Remote.pitch-1500)+pitch_offset)*roll_pitch_ratio;	 //将遥杆值作为飞行角度的期望值
										pidRoll.desired = (-(Remote.roll-1500)+roll_offset)*roll_pitch_ratio;	
							}

							{
									const float yaw_ratio =   0.5f;		//偏航角灵敏度度
							
									if(Remote.yaw>1820)
									{
										pidYaw.desired -= yaw_ratio;	//符号控制偏航角方向
									}
									else if(Remote.yaw <1180)
									{
										pidYaw.desired += yaw_ratio;	//符号控制偏航角方向
									}		
							}							
				}
				remote_unlock();
			
		}
  }
    /* Real 3 ms clock replaces a loop-iteration timeout. The ISR also checks
     * RC freshness, so a stalled polling loop cannot sustain automatic flight. */
    if ((uint32_t)(SysTick_count - last_valid_tick) > AUTO_RC_LOSS_TICKS &&
        (uint32_t)(SysTick_count - last_reinit_tick) > AUTO_RC_LOSS_TICKS) {
        ALL_flag.unlock = 0;
        last_reinit_tick = SysTick_count;
        NRF24L01_init();
    }
}

/*****************************************************************************************
 *  解锁判断
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
#include "imu.h"
void Remote_ArmForAutoFlight(void)
{
    last_Remote_AUX1 = Remote.AUX1;
    roll_offset = Remote.roll - 1500;
    pitch_offset = Remote.pitch - 1500;
    pidRoll.offset = -roll_offset * 0.04f;
    pidPitch.offset = -pitch_offset * 0.04f;
    pidRoll.desired = pidPitch.desired = 0.0f;
    imu_rest();
}

void remote_unlock(void)
{
	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0;

    if (AutoFlight_Active()) {
        last_Remote_AUX1 = Remote.AUX1;
        status = PROCESS_31;
        cnt = 0;
        if (Remote.thr < 1050U && Remote.yaw < 1150U) ALL_flag.unlock = 0;
        return;
    }
    if (!ALL_flag.unlock && status == PROCESS_31) status = WAITING_1;
	if(Remote.thr<1050 &&Remote.yaw<1150)                         //油门遥杆左下角锁定
	{
		status = EXIT_255;
	}
	
	switch(status)
	{
		case WAITING_1://等待解锁
			if(Remote.thr<1150)           //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁
			{			 
					 status = WAITING_2;				 
			}		
			break;
		case WAITING_2:
			if(Remote.thr>1800)          
			{		
						static uint8_t cnt = 0;
					 	cnt++;		
						if(cnt>5) //最高油门需保持200ms以上，防止遥控开机初始化未完成的错误数据
						{	
								cnt=0;
								status = WAITING_3;
						}
			}			
			break;
		case WAITING_3:
			if(Remote.thr<1100)          
			{			 
					 status = WAITING_4;				 
			}			
			break;			
		case WAITING_4:	//解锁前准备	  
						
				ALL_flag.unlock = 1;
				status = PROCESS_31;
				LED.status = AlwaysOn;	
				last_Remote_AUX1=Remote.AUX1;
				ALL_flag.height_lock = 0;//必须在姿态模式下起飞
				ALL_flag.flow_control = 0;//必须在姿态模式下起飞	
				pidRoll.offset =   -(Remote.roll-1500)*0.04;//微调值做前馈
				pidPitch.offset  = -(Remote.pitch-1500)*0.04;
				roll_offset = (Remote.roll-1500); //记录微调值
				pitch_offset = (Remote.pitch-1500);
						imu_rest();	
				 break;		
		case PROCESS_31:	//进入解锁状态
				if(Remote.thr<1020)
				{
					if(!ALL_flag.height_lock )
					{
						if(cnt++ > 3000)                                     // 油门遥杆处于最低9S自动上锁
						{								
							status = EXIT_255;								
						}
					}
				}
				else if(!ALL_flag.unlock)                           //Other conditions lock 
				{
					status = EXIT_255;				
				}
				else					
					cnt = 0;
			break;
		case EXIT_255: //进入锁定
			LED.status = AllFlashLight;	                                 //exit
			cnt = 0;
			LED.FlashTime = 100; //100*3ms		
			ALL_flag.unlock = 0;
			status = WAITING_1;
			break;
		default:
			status = EXIT_255;
			break;
	}
}
/***********************END OF FILE*************************************/







