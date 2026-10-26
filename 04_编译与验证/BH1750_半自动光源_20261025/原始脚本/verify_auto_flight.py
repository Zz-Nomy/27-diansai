"""Compile actual flight/receiver/parser/mixer/LED sources against hardware mocks.
No attached device is opened. The plant model is illustrative, not flight proof.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys

sys.stdout.reconfigure(encoding='utf-8')
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1] / '02_飞控源码与说明' / '02主控源码'
HOST = HERE / 'host_tests'
HOST.mkdir(exist_ok=True)
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cc',default=r'C:/mingw64/bin/gcc.exe')
args=parser.parse_args()
CC = Path(args.cc)
env = os.environ.copy()
env['PATH'] = str(CC.parent)+os.pathsep+env.get('PATH','')

def source(name):
    data = (ROOT/name).read_bytes()
    try: text = data.decode('utf-8')
    except UnicodeDecodeError: text = data.decode('gbk')
    return re.sub(r'^\s*#include[^\n]*', '', text, flags=re.M)

prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#undef assert
#define assert(condition) do { if (!(condition)) { \
    fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#condition); exit(2); \
} } while (0)
#include "ALL_DATA.h"
#include "auto_flight_config.h"
#define LIMIT(x,lo,hi) ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
#define Gyro_G 0.06103515625f
#define FAILED 1
typedef struct { uint32_t CCR1,CCR2,CCR3,CCR4; } MockTimer;
static MockTimer motor_timer, led_timer;
#define TIM2 (&motor_timer)
#define TIM3 (&led_timer)
volatile uint32_t SysTick_count;
_st_Mpu MPU6050;
_st_Remote Remote;
_st_AngE Angle;
_st_ALL_flag ALL_flag;
PidObject pidRateX,pidRateY,pidRateZ,pidRoll,pidPitch,pidYaw;
PidObject pidHeightRate,pidHeightHigh,pidPosRateX,pidPosRateY,pidPositionX,pidPositionY;
int16_t motor_PWM_Value[4];
static uint8_t mock_sensor[5], mock_packet[32];
static int mock_bus_fails, packet_available, radio_reinits;
static int IIC_read_Bytes(uint8_t addr,uint8_t reg,uint8_t *out,uint8_t n)
{
    assert(addr==0xC4 && reg==0x33 && n==5);
    if (mock_bus_fails) return 1;
    memcpy(out,mock_sensor,5); return 0;
}
static int NRF24L01_RxPacket(uint8_t *out)
{
    if (!packet_available) return 1;
    packet_available=0; memcpy(out,mock_packet,32); return 0;
}
static void NRF24L01_init(void) { ++radio_reinits; }
static void imu_rest(void) { Angle.roll=Angle.pitch=0; }
static float GetNormAccz(void) { return 16384.0f; }
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t mask) { (void)mask; }
void Remote_ArmForAutoFlight(void);
'''

names = ['HARDWARE/flow.h','HAL/control.h','MATH/pid.h','HARDWARE/LED.h',
         'HAL/auto_flight.h','MATH/pid.c','HAL/auto_flight.c','HARDWARE/flow.c',
         'HAL/remote.c','HAL/control.c','HARDWARE/LED.c']
combined = prefix + '\n'.join(source(name) for name in names)
# Exercise the actual I2C GPIO-read macros without a physical GPIO block.
i2c_macros='\n'.join(line for line in (ROOT/'CONFIG/I2C.h').read_bytes().decode('gbk').splitlines()
                     if re.match(r'#define\s+(SDA_read|SCL_read)\b',line))
combined += '''
static struct { uint32_t IDR; } gpio_fixture;
#define GPIOB (&gpio_fixture)
#define GPIO_Pin_6 64U
#define GPIO_Pin_7 128U
''' + i2c_macros
combined += '\n' + (HERE/'host_scenarios.c').read_text(encoding='utf-8')
(HOST/'actual_sources_test.c').write_text(combined,encoding='utf-8')
reports=[]
for variant, defines in [('cm_latched', []), ('mm_momentary', ['-DFLOW_HEIGHT_MM_PER_UNIT=1','-DAUTO_AUX3_LATCHED=0'])]:
    binary=HOST/(variant+'.exe')
    cmd=[str(CC),'-std=c99','-O1','-Wall','-Wextra','-Wno-unused-parameter',
         '-I'+str(ROOT/'USER'),'-I'+str(ROOT/'HAL'),*defines,
         'actual_sources_test.c','-o',binary.name,'-lm']
    build=subprocess.run(cmd,cwd=HOST,env=env,capture_output=True,text=True,errors='replace')
    (HOST/(variant+'_compile.txt')).write_text(build.stdout+build.stderr,encoding='utf-8')
    if build.returncode:
        print(build.stdout+build.stderr)
        raise SystemExit(build.returncode)
    for case in ['broken_sensor','button','hover_model','zero_and_fault','alarms',
                 'landing_model','rc_timeout','takeover','tilt','flow_loss',
                 'reject_conditions','manual_legacy','signed_and_range',
                 'i2c_low_line','alarm_boundaries','ground_sensor_loss']:
        r=subprocess.run([str(binary),case],cwd=HOST,env=env,capture_output=True,text=True,errors='replace',timeout=15)
        reports.append({'variant':variant,'case':case,'returncode':r.returncode,
                        'stdout':r.stdout,'stderr':r.stderr})
        print(variant,case,'PASS' if r.returncode==0 else 'FAIL',r.stdout.strip(),r.stderr.strip())
result={'scope':'Actual changed C functions with mocked hardware; toy vertical plant is not a flight test',
        'passed':all(r['returncode']==0 for r in reports),'cases':reports,
        'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in names}}
(HOST/'test_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
raise SystemExit(0 if result['passed'] else 1)
