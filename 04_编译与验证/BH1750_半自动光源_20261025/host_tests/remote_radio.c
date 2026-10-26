
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define assert(c) do { if(!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(2); } } while(0)

typedef uint8_t u8;
#define TX_PLOAD_WIDTH 32
#define TX_ADR_WIDTH 5
#define SPI_WRITE_REG 0x20
#define STATUS 7
#define FLUSH_TX 0xE1
#define WR_TX_PLOAD 0xA0
#define TX_ADDR 0x10
#define NCONFIG 0
#define MAX_TX 0x10
#define TX_OK 0x20
#define SUCCESS 0
#define FAILED 1
static uint8_t TX_ADDRESS[5]={1,2,3,4,5};
static uint32_t mock_us, mock_ready_us=900;
static int ce, config, flushes, never_ready;
static uint8_t final_status=TX_OK;
#define Clr_NRF24L01_CE ce=0
#define Set_NRF24L01_CE ce=1
#define READ_NRF24L01_IRQ (never_ready || mock_us<mock_ready_us)
static uint32_t GetSysTime_us(void) { mock_us+=50; return mock_us; }
static uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value) {
    if(reg==(SPI_WRITE_REG+NCONFIG)) config=value;
    if(reg==FLUSH_TX)++flushes;
    return 0;
}
static uint8_t NRF24L01_Write_Buf(uint8_t reg,uint8_t *data,uint8_t n) { return 0; }
static uint8_t NRF24L01_Read_Reg(uint8_t reg) { return final_status; }
static void RX_Mode(void) { config=0x0F; ce=1; }
static void TX_Mode(void) { config=0x0E; ce=0; }
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 state;
    uint32_t start_us = GetSysTime_us();
	Clr_NRF24L01_CE;
    NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, 0x70);
    NRF24L01_Write_Reg(FLUSH_TX, 0xFF);
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
	Set_NRF24L01_CE;                                     //启动发送	   
    while (READ_NRF24L01_IRQ != 0) {
        if ((uint32_t)(GetSysTime_us() - start_us) >= 2500U) {
            Clr_NRF24L01_CE;
            NRF24L01_Write_Reg(FLUSH_TX, 0xFF);
            return 0xFF;
        }
    }
    Clr_NRF24L01_CE;
	state=NRF24L01_Read_Reg(STATUS);                     //读取状态寄存器的值	   
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);      //清除TX_DS或MAX_RT中断标志
	if(state&MAX_TX)                                     //达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);               //清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(state&TX_OK)                                      //发送完成
	{
		return TX_OK;
	}
	return 0xff;                                         //其他原因发送失败
}void NRF24L01_Listen(void)
{
    Clr_NRF24L01_CE;
    NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0F);
    Set_NRF24L01_CE;
}
int main(int argc,char **argv) {
    uint8_t data[32]={0},result;
    if(!strcmp(argv[1],"timeout_returns_to_rx")) never_ready=1;
    if(!strcmp(argv[1],"max_retry_returns_to_rx")) final_status=MAX_TX;
    if(!strcmp(argv[1],"spurious_irq_returns_to_rx")) final_status=0;
    TX_Mode(); result=NRF24L01_TxPacket(data); NRF24L01_Listen();
    assert(config==0x0F && ce==1 && mock_us<4000);
    if(!strcmp(argv[1],"success_returns_to_rx")) assert(result==TX_OK);
    if(!strcmp(argv[1],"timeout_returns_to_rx")) assert(result==0xFF && flushes>=2);
    if(!strcmp(argv[1],"max_retry_returns_to_rx")) assert(result==MAX_TX);
    if(!strcmp(argv[1],"spurious_irq_returns_to_rx")) assert(result==0xFF);
    puts("passed");return 0;
}
