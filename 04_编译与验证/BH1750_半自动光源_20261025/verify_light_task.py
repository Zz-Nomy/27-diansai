"""Actual C regression against sensor/radio/GPIO mocks. Never opens hardware."""
from pathlib import Path
import argparse
import ast
import hashlib
import json
import os
import re
import subprocess
import sys

sys.stdout.reconfigure(encoding='utf-8')
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
FLIGHT=ROOT/'02_飞控源码与说明'/'02主控源码'
REMOTE=ROOT/'02_飞控源码与说明'/'03遥控源码'
OUT=HERE/'host_tests'
OUT.mkdir(exist_ok=True)
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cc',default='C:/mingw64/bin/gcc.exe')
args=parser.parse_args()
CC=Path(args.cc)
env=os.environ.copy(); env['PATH']=str(CC.parent)+os.pathsep+env.get('PATH','')

def original(base,name):
    data=(base/name).read_bytes()
    try: return data.decode('utf-8')
    except UnicodeDecodeError: return data.decode('gbk')
def source(base,name):
    return re.sub(r'^\s*#include[^\n]*','',original(base,name),flags=re.M)
def function(text,name):
    m=re.search(r'(?:void|uint8_t|uint32_t|int8_t|u8)\s+'+name+r'\s*\([^;]*?\)\s*\{',text)
    assert m,name
    start=m.start(); i=m.end(); depth=1
    while depth:
        if text[i]=='{': depth+=1
        elif text[i]=='}': depth-=1
        i+=1
    return text[start:i]

reports=[]
def compile_run(label,code,cases,defines=()):
    c=OUT/(label+'.c'); exe=OUT/(label+'.exe')
    c.write_text(code,encoding='utf-8')
    cmd=[str(CC),'-std=c99','-O1','-Wall','-Wextra','-Wno-unused-parameter',
         '-I'+str(FLIGHT/'USER'),'-I'+str(FLIGHT/'HAL'),*defines,str(c),'-o',str(exe),'-lm']
    r=subprocess.run(cmd,env=env,capture_output=True,text=True,errors='replace',timeout=45)
    (OUT/(label+'_compile.txt')).write_text(r.stdout+r.stderr,encoding='utf-8')
    if r.returncode: print(r.stdout+r.stderr); raise SystemExit(r.returncode)
    for case in cases:
        r=subprocess.run([str(exe),case],env=env,capture_output=True,text=True,errors='replace',timeout=20)
        reports.append({'variant':label,'case':case,'returncode':r.returncode,'stdout':r.stdout,'stderr':r.stderr})
        print(label,case,'PASS' if not r.returncode else 'FAIL',r.stderr.strip())

legacy=ast.parse((HERE/'原始脚本'/'verify_auto_flight.py').read_text(encoding='utf-8'))
prefix=next(n.value.value for n in legacy.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='prefix' for t in n.targets))
prefix+='''
#include "light_task_config.h"
static uint16_t mock_light_raw=360;
static int mock_light_bus_fails, mock_light_reads;
static uint8_t mock_commands[1024], mock_reply[32];
static unsigned mock_command_count, mock_reply_count;
static int IIC_Send_Command(uint8_t address,uint8_t cmd) {
    assert(address==0x46); assert(mock_command_count<sizeof(mock_commands));
    mock_commands[mock_command_count++]=cmd;
    return mock_light_bus_fails;
}
static int IIC_Receive_Bytes(uint8_t address,uint8_t *out,uint8_t n) {
    assert(address==0x46 && n==2); ++mock_light_reads;
    if(mock_light_bus_fails) return 1;
    out[0]=(uint8_t)(mock_light_raw>>8); out[1]=(uint8_t)mock_light_raw; return 0;
}
static uint8_t NRF24L01_SendReply(uint8_t *packet) {
    memcpy(mock_reply,packet,32); ++mock_reply_count; return 0;
}
'''
names=['HARDWARE/flow.h','HAL/control.h','MATH/pid.h','HARDWARE/LED.h','HAL/auto_flight.h',
       'HARDWARE/bh1750.h','HAL/light_task.h','HAL/light_link_protocol.h',
       'MATH/pid.c','HAL/auto_flight.c','HARDWARE/flow.c','HARDWARE/bh1750.c','HAL/light_task.c',
       'HAL/light_link.c','HAL/remote.c','HAL/control.c','HARDWARE/LED.c']
combined=prefix+'\n'.join(source(FLIGHT,n) for n in names)
i2c_macros='\n'.join(line for line in original(FLIGHT,'CONFIG/I2C.h').splitlines()
                    if re.match(r'#define\s+(SDA_read|SCL_read)\b',line))
combined+='''
static struct { uint32_t IDR; } gpio_fixture;
#define GPIOB (&gpio_fixture)
#define GPIO_Pin_6 64U
#define GPIO_Pin_7 128U
'''+i2c_macros
scenarios=(HERE/'原始脚本'/'host_scenarios.c').read_text(encoding='utf-8')
scenarios=scenarios.replace('    AutoFlight_Tick(0.006f);','    BH1750_Tick(0.006f);\n    AutoFlight_Tick(0.006f);\n    LightTask_Tick(0.006f);')
scenarios=scenarios.replace('    AutoFlight_Init();','    AutoFlight_Init();\n    BH1750_Init();\n    LightTask_Init();')
scenarios=scenarios.replace('int main(int argc,char **argv)',(HERE/'light_scenarios.c').read_text(encoding='utf-8')+'\nint main(int argc,char **argv)')
scenarios=scenarios.replace('name=argv[1]; setup();','name=argv[1]; setup();\n    if (run_light_case(name)) return 0;')
combined+='\n'+scenarios
old_cases=['broken_sensor','button','hover_model','zero_and_fault','alarms','landing_model','rc_timeout',
           'takeover','tilt','flow_loss','reject_conditions','manual_legacy','signed_and_range',
           'i2c_low_line','alarm_boundaries','ground_sensor_loss']
light_cases=re.findall(r'!strcmp\(name,"(light_[^"]+)"\)',(HERE/'light_scenarios.c').read_text(encoding='utf-8'))
for variant,defines in [('cm_latched',[]),('mm_momentary',['-DFLOW_HEIGHT_MM_PER_UNIT=1','-DAUTO_AUX3_LATCHED=0'])]:
    compile_run(variant,combined,old_cases+light_cases,defines)

basic=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define assert(c) do { if(!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(2); } } while(0)
'''
feedback=basic+source(FLIGHT,'HAL/light_link_protocol.h')+'''
static int beep_pin, send_count;
volatile uint32_t SysTick_count;
#define BEEP_H beep_pin=1
#define BEEP_L beep_pin=0
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t value) {}
static uint8_t NRF24L01_RxPacket(uint8_t *out) { return 1; }
static void RC_Analy(void) { ++send_count; }
'''+source(REMOTE,'HAL/light_feedback.h')+source(REMOTE,'HAL/light_feedback.c')+r'''
int main(int argc,char **argv) {
    uint8_t p[32]; int i,rises=0,prev=0,total=0;
    LightLinkStatus s={0};
    RemoteFeedback_Init(); s.event_id=1; s.flags=LIGHT_LINK_ALERT; s.state=3;
    LightLink_Encode(p,&s);
    if(!strcmp(argv[1],"beep_three_seconds")) {
        for(i=0;i<4000;++i) {
            if(i%20==0) { s.alert_elapsed_ms=i<3000?i:3000; if(i>=3000)s.flags=0; LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,i)); }
            RemoteFeedback_BuzzerTick(i);
            if(beep_pin && !prev)++rises; prev=beep_pin; total+=beep_pin;
        }
        assert(rises==3 && total==450 && !beep_pin);
    } else if(!strcmp(argv[1],"duplicate_no_extension")) {
        for(i=0;i<4000;++i) {
            if(i%20==0) assert(RemoteFeedback_Accept(p,i));
            RemoteFeedback_BuzzerTick(i);
            if(beep_pin && !prev)++rises; prev=beep_pin;
        }
        assert(rises==3 && !beep_pin);
    } else if(!strcmp(argv[1],"bad_feedback")) {
        p[31]++; assert(!RemoteFeedback_Accept(p,0));
        RemoteFeedback_BuzzerTick(1); assert(!beep_pin);
    } else if(!strcmp(argv[1],"late_feedback")) {
        s.alert_elapsed_ms=1100; LightLink_Encode(p,&s);
        assert(RemoteFeedback_Accept(p,5000));
        RemoteFeedback_BuzzerTick(5000); assert(beep_pin);
        RemoteFeedback_BuzzerTick(5050); assert(!beep_pin);
        RemoteFeedback_BuzzerTick(5900); assert(beep_pin);
        RemoteFeedback_BuzzerTick(6900); assert(!beep_pin);
    } else if(!strcmp(argv[1],"key_click_priority")) {
        assert(RemoteFeedback_Accept(p,0)); RemoteFeedback_KeyClick(500);
        RemoteFeedback_BuzzerTick(500); assert(!beep_pin);
        RemoteFeedback_KeyClick(3100); RemoteFeedback_BuzzerTick(3100); assert(beep_pin);
        RemoteFeedback_BuzzerTick(3130); assert(!beep_pin);
    } else if(!strcmp(argv[1],"remote_poll_period")) {
        for(i=0;i<1000;++i) { SysTick_count=i; RemoteControl_Poll(); }
        assert(send_count==50);
    } else if(!strcmp(argv[1],"clock_wrap")) {
        assert(RemoteFeedback_Accept(p,0xFFFFFF00U));
        RemoteFeedback_BuzzerTick(0xFFFFFF00U); assert(beep_pin);
        RemoteFeedback_BuzzerTick(0x00000AB8U); assert(!beep_pin);
    } else if(!strcmp(argv[1],"takeoff_reject_beeps")) {
        s.event_id=0; s.flags=0; s.state=0;
        s.diagnostic_valid=1; s.auto_error=3; s.start_request_id=1;
        s.range_raw=0; s.height_mm=60; s.aux3=2000;
        LightLink_Encode(p,&s);
        for(i=0;i<2000;++i) {
            if(i%20==0) assert(RemoteFeedback_Accept(p,i));
            RemoteFeedback_BuzzerTick(i);
            if(beep_pin && !prev)++rises; prev=beep_pin; total+=beep_pin;
        }
        assert(rises==3 && total==360 && !beep_pin);
        assert(remote_light_status.auto_error==3 && remote_light_status.aux3==2000);
    } else if(!strcmp(argv[1],"takeoff_success_no_reject_beep")) {
        s.flags=0; s.diagnostic_valid=1; s.start_request_id=1; s.auto_state=1;
        LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,0));
        RemoteFeedback_BuzzerTick(1); assert(!beep_pin);
    } else if(!strcmp(argv[1],"bad_diagnostic")) {
        s.flags=0; s.diagnostic_valid=1; s.auto_error=255;
        LightLink_Encode(p,&s); assert(!RemoteFeedback_Accept(p,0));
        RemoteFeedback_BuzzerTick(1); assert(!beep_pin);
    } else if(!strcmp(argv[1],"legacy_feedback_diagnostic_reset")) {
        s.flags=0; s.diagnostic_valid=1; s.start_request_id=1; s.auto_state=1;
        LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,0));
        s.diagnostic_valid=0; LightLink_Encode(p,&s);
        assert(RemoteFeedback_Accept(p,20));
        assert(!remote_light_status.diagnostic_valid && !remote_light_status.start_request_id);
    } else assert(0);
    puts("passed");return 0;
}
'''
compile_run('remote_feedback',feedback,['beep_three_seconds','duplicate_no_extension','bad_feedback',
                                      'late_feedback','key_click_priority','remote_poll_period','clock_wrap',
                                      'takeoff_reject_beeps','takeoff_success_no_reject_beep',
                                      'bad_diagnostic','legacy_feedback_diagnostic_reset'])

i2c=basic+'''
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
'''+original(FLIGHT,'CONFIG/IIC_SOFTWARE.c').split('/* Raw transactions for BH1750:',1)[1].split('*/',1)[1]+r'''
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
'''
compile_run('i2c_raw',i2c,['raw_write','raw_read','command_nack','address_nack','start_failure','invalid_arguments'])

timing=basic+'''
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
'''+source(FLIGHT,'USER/delay.c')+source(FLIGHT,'USER/scheduler.c')+r'''
int main(int argc,char **argv) {
    int ms; cycleCounterInit();
    for(ms=1;ms<=3000;++ms) { SysTick_Handler(); if(ms%3==0) TIM1_UP_IRQHandler(); }
    assert(SysTick_count==1000 && laser_calls==500 && light_calls==500);
    assert(fabsf(light_seconds-3.0f)<0.0001f);
    assert(GetSysTime_us()==3000000U);
    puts("passed");return 0;
}
'''
compile_run('timebase',timing,['separate_1ms_3ms_clocks'])

# Verify the actual transmitter packet builder, including all eight channels.
sender=basic+source(FLIGHT,'HAL/light_link_protocol.h')+'''
static struct { uint16_t roll,pitch,thr,yaw,AUX1,AUX2,AUX3,AUX4; } Remote;
static uint8_t tx_data[32]={0xAA,0xAF,3,27}, sent_packet[32];
static int tx_mode, listening;
static void TX_Mode(void) { ++tx_mode; }
static uint8_t NRF24L01_TxPacket(uint8_t *p) { memcpy(sent_packet,p,32); return 0; }
static void NRF24L01_Listen(void) { ++listening; }
'''+function(original(REMOTE,'HAL/remote.c'),'NRF_SEND')+r'''
int main(int argc,char **argv) {
    uint16_t c[8]={1500,1600,1000,1400,1000,2000,2000,1000};
    int i; memcpy(&Remote,c,sizeof c); NRF_SEND();
    for(i=0;i<8;++i) assert(LightLink_Get16(sent_packet+4+2*i)==c[i]);
    assert(LightLink_Requested(sent_packet) && sent_packet[31]==LightLink_Checksum(sent_packet));
    for(i=24;i<31;++i) assert(sent_packet[i]==0);
    assert(tx_mode==1 && listening==1); puts("passed");return 0;
}
'''
compile_run('remote_sender',sender,['eight_channels_and_feedback_request'])

for board,base in [('flight',FLIGHT),('remote',REMOTE)]:
    radio=basic+'''
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
'''+function(original(base,'HARDWARE/nrf24l01.c'),'NRF24L01_TxPacket')
    if board=='flight':
        radio+=function(original(base,'HARDWARE/nrf24l01.c'),'NRF24L01_SendReply')
        send='result=NRF24L01_SendReply(data);'
    else:
        radio+=function(original(base,'HARDWARE/nrf24l01.c'),'NRF24L01_Listen')
        send='TX_Mode(); result=NRF24L01_TxPacket(data); NRF24L01_Listen();'
    radio+='''
int main(int argc,char **argv) {
    uint8_t data[32]={0},result;
    if(!strcmp(argv[1],"timeout_returns_to_rx")) never_ready=1;
    if(!strcmp(argv[1],"max_retry_returns_to_rx")) final_status=MAX_TX;
    if(!strcmp(argv[1],"spurious_irq_returns_to_rx")) final_status=0;
    '''+send+'''
    assert(config==0x0F && ce==1 && mock_us<4000);
    if(!strcmp(argv[1],"success_returns_to_rx")) assert(result=='''+('0' if board=='flight' else 'TX_OK')+''');
    if(!strcmp(argv[1],"timeout_returns_to_rx")) assert(result==0xFF && flushes>=2);
    if(!strcmp(argv[1],"max_retry_returns_to_rx")) assert(result==MAX_TX);
    if(!strcmp(argv[1],"spurious_irq_returns_to_rx")) assert(result=='''+('FAILED' if board=='flight' else '0xFF')+''');
    puts("passed");return 0;
}
'''
    compile_run(board+'_radio',radio,['success_returns_to_rx','timeout_returns_to_rx',
                                     'max_retry_returns_to_rx','spurious_irq_returns_to_rx'])

remote_macros='\n'.join(line for line in original(REMOTE,'CONFIG/I2C.h').splitlines()
                        if re.match(r'#define\s+(SDA_read|SCL_read)\b',line))
macro_check=basic+'''
static struct { uint32_t IDR; } port;
#define GPIOB (&port)
#define GPIO_Pin_6 64U
#define GPIO_Pin_7 128U
'''+remote_macros+'''
int main(int argc,char **argv) {
    port.IDR=1; assert(!SDA_read && !SCL_read);
    port.IDR=129; assert(SDA_read && !SCL_read);
    port.IDR=65; assert(SCL_read && !SDA_read);
    puts("passed");return 0;
}
'''
compile_run('remote_i2c_macros',macro_check,['low_line_detection'])

assert (FLIGHT/'HAL/light_link_protocol.h').read_bytes()==(REMOTE/'HAL/light_link_protocol.h').read_bytes()
result={'scope':'Actual C with mocked sensors/radio/GPIO; no physical flight or RF proof',
        'passed':all(r['returncode']==0 for r in reports),'case_count':len(reports),'cases':reports,
        'source_sha256':{n:hashlib.sha256((FLIGHT/n).read_bytes()).hexdigest() for n in names}}
(OUT/'test_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print('PASS' if result['passed'] else 'FAIL',len(reports),'checks')
raise SystemExit(0 if result['passed'] else 1)
