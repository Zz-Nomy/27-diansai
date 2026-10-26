
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define assert(c) do { if(!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(2); } } while(0)

#define SUCCESS 0
#define FAILED 1
static int starts, stops, ack_count, nack_count, sent_count, wait_count, fail_at, fail_start;
static uint8_t sent[8];
static uint8_t I2c_Start(void) { ++starts; return fail_start; }
static void I2c_Stop(void) { ++stops; }
static void I2c_SendByte(uint8_t byte) { sent[sent_count++]=byte; }
static uint8_t I2c_WaitAck(void) { return ++wait_count==fail_at; }
static uint8_t I2c_ReadByte(void) { return 0x42; }
static void I2c_Ack(void) { ++ack_count; }
static void I2c_NoAck(void) { ++nack_count; }

int8_t IIC_Send_Command(uint8_t addr, uint8_t command)
{
    if (addr & 1U) return FAILED;
    if (I2c_Start() == FAILED) { I2c_Stop(); return FAILED; }
    I2c_SendByte(addr);
    if (I2c_WaitAck() == FAILED) { I2c_Stop(); return FAILED; }
    I2c_SendByte(command);
    if (I2c_WaitAck() == FAILED) { I2c_Stop(); return FAILED; }
    I2c_Stop();
    return SUCCESS;
}
int8_t IIC_Receive_Bytes(uint8_t addr, uint8_t *data, uint8_t len)
{
    if ((addr & 1U) || !data || !len) return FAILED;
    if (I2c_Start() == FAILED) { I2c_Stop(); return FAILED; }
    I2c_SendByte(addr | 1U);
    if (I2c_WaitAck() == FAILED) { I2c_Stop(); return FAILED; }
    while (len) {
        *data++ = I2c_ReadByte();
        if (--len) I2c_Ack(); else I2c_NoAck();
    }
    I2c_Stop();
    return SUCCESS;
}

int main(int argc,char **argv) {
    uint8_t bytes[2];
    if(!strcmp(argv[1],"raw_write")) {
        assert(IIC_Send_Command(0x46,0x10)==0);
        assert(starts==1 && stops==1 && sent_count==2 && sent[0]==0x46 && sent[1]==0x10);
    } else if(!strcmp(argv[1],"raw_read")) {
        assert(IIC_Receive_Bytes(0x46,bytes,2)==0);
        assert(sent_count==1 && sent[0]==0x47 && ack_count==1 && nack_count==1 && stops==1);
        assert(bytes[0]==0x42 && bytes[1]==0x42);
    } else if(!strcmp(argv[1],"command_nack")) {
        fail_at=2; assert(IIC_Send_Command(0x46,0x10)==1 && stops==1);
    } else if(!strcmp(argv[1],"address_nack")) {
        fail_at=1; assert(IIC_Receive_Bytes(0x46,bytes,2)==1 && stops==1);
    } else if(!strcmp(argv[1],"start_failure")) {
        fail_start=1; assert(IIC_Send_Command(0x46,0x10)==1 && stops==1);
    } else if(!strcmp(argv[1],"invalid_arguments")) {
        assert(IIC_Send_Command(0x47,0x10)==1);
        assert(IIC_Receive_Bytes(0x46,0,2)==1 && IIC_Receive_Bytes(0x46,bytes,0)==1);
        assert(!starts);
    } else assert(0);
    puts("passed");return 0;
}
