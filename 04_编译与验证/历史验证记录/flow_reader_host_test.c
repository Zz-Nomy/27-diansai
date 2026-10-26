
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint16_t u16;
typedef uint32_t u32;
#define FAILED 1
struct { int8_t vel_x, vel_y; int32_t pos_x_i, pos_y_i; uint8_t squal; } flow_data;
struct { uint8_t height_lock, flow_control; } ALL_flag;
uint32_t VL53L01_high;
uint8_t temp_data[6], error_count;
static uint8_t frame[5];
static int bus_fails;
static int8_t IIC_read_Bytes(uint8_t addr, uint8_t reg, uint8_t *out, uint8_t len)
{
    assert(addr == 0xC4 && reg == 0x33 && len == 5);
    if (bus_fails) return FAILED;
    memcpy(out, frame, 5);
    return 0;
}
void flow_get_data(void)
//----------------------------------------------	
{	//获取当前激光高度数据  LC08激光光流模块

	#define LC08_ADDRESS 0xC4 //模块地址 最低位读写标志位 0：写  1：读
	#define READ_ADDRESS 0X33 //高度数据寄存器地址
	if(IIC_read_Bytes(LC08_ADDRESS,READ_ADDRESS,temp_data,5) != FAILED)//如果读取成功
	{
			u16 high;
			flow_data.vel_x =(*(int8_t*)&temp_data[2]);
			flow_data.vel_y = (*(int8_t*)&temp_data[3]);
			//光流数据积分得出位移
			flow_data.pos_x_i += flow_data.vel_x;
			flow_data.pos_y_i += flow_data.vel_y;
			flow_data.squal = temp_data[4];
			 high = ((u32)temp_data[0]<<8) | ((u16)temp_data[1]) ;   //激光高度
			if(high!=0)
				VL53L01_high=high;
			error_count=0;
	}
	else
	{
			if(error_count<10)//如果通信失败，则要退出定高定点
			{
				error_count++;
			}
			else
			{
					ALL_flag.height_lock=0;
					ALL_flag.flow_control=0;
			}
	}
	
}	


















static void reset_fixture(void)
{
    memset(&flow_data, 0, sizeof(flow_data));
    memset(frame, 0, sizeof(frame));
    VL53L01_high = error_count = bus_fails = 0;
    ALL_flag.height_lock = ALL_flag.flow_control = 1;
}
int main(void)
{
    int i;
    reset_fixture();
    frame[0] = 0x01; frame[1] = 0x23;
    frame[2] = 0xFE; frame[3] = 0x80; frame[4] = 31;
    flow_get_data();
    assert(VL53L01_high == 0x0123);
    assert(flow_data.vel_x == -2 && flow_data.vel_y == -128);
    assert(flow_data.pos_x_i == -2 && flow_data.pos_y_i == -128);
    assert(flow_data.squal == 31);
    puts("OBSERVATION 1: address/register/length and signed-byte parsing confirmed");

    reset_fixture();
    frame[0] = 0x03; frame[1] = 0x20;
    flow_get_data();
    assert(VL53L01_high == 800);
    frame[0] = frame[1] = 0;
    for (i = 0; i < 20; ++i) flow_get_data();
    assert(VL53L01_high == 800 && error_count == 0);
    assert(ALL_flag.height_lock == 1 && ALL_flag.flow_control == 1);
    puts("OBSERVATION 2: 20 successful zero-height frames retain old 800 reading and enabled modes");

    reset_fixture();
    bus_fails = 1;
    for (i = 0; i < 10; ++i) flow_get_data();
    assert(error_count == 10);
    assert(ALL_flag.height_lock == 1 && ALL_flag.flow_control == 1);
    flow_get_data();
    assert(ALL_flag.height_lock == 0 && ALL_flag.flow_control == 0);
    puts("OBSERVATION 3: consecutive bus failures disable modes on call 11");

    bus_fails = 0;
    frame[0] = 0x03; frame[1] = 0x20;
    flow_get_data();
    assert(error_count == 0 && VL53L01_high == 800);
    assert(ALL_flag.height_lock == 0 && ALL_flag.flow_control == 0);
    puts("OBSERVATION 4: success clears error counter without automatically reenabling modes");
    return 0;
}
