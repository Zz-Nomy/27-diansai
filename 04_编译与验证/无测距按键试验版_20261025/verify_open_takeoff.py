"""Run the independent version's actual C parser, state machine and mixer with mocks."""
from pathlib import Path
import ast, hashlib, json, os, re, subprocess, sys
sys.stdout.reconfigure(encoding='utf-8')
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
VERSION=ROOT/'02_飞控源码与说明'/'04_无测距按键起飞试验版_20261025'
FLIGHT=VERSION/'飞控'; REMOTE=VERSION/'遥控器'
OUT=HERE/'host_tests'; OUT.mkdir(exist_ok=True)
CC=Path('C:/mingw64/bin/gcc.exe')
env=os.environ.copy(); env['PATH']=str(CC.parent)+os.pathsep+env.get('PATH','')
def read(p):
    b=p.read_bytes()
    try: return b.decode('utf-8-sig')
    except UnicodeDecodeError: return b.decode('gbk')
def source(base,name): return re.sub(r'^\s*#include[^\n]*','',read(base/name),flags=re.M)
legacy=ast.parse((ROOT/'04_编译与验证/BH1750_半自动光源_20261025/原始脚本/verify_auto_flight.py').read_text(encoding='utf-8'))
prefix=next(n.value.value for n in legacy.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='prefix' for t in n.targets))
prefix=prefix.replace('static int mock_bus_fails, packet_available, radio_reinits;', 'static int mock_bus_fails, packet_available, radio_reinits, range_reads;')
prefix=prefix.replace('assert(addr==0xC4 && reg==0x33 && n==5);','++range_reads; assert(addr==0xC4 && reg==0x33 && n==5);')
prefix+='\nstatic uint8_t NRF24L01_SendReply(uint8_t *p) { return 0; }\n'
names=['HAL/open_takeoff_config.h','HAL/control.h','MATH/pid.h','HARDWARE/LED.h','HARDWARE/flow.h',
       'HARDWARE/bh1750.h','HAL/light_task.h','HAL/light_link_protocol.h','HAL/auto_flight.h',
       'MATH/pid.c','HAL/light_task.c','HAL/auto_flight.c','HARDWARE/flow.c',
       'HAL/light_link.c','HAL/remote.c','HAL/control.c','HARDWARE/LED.c']
combined=prefix
for name in names:
    combined+=source(FLIGHT,name)+'\n'
    if name=='HARDWARE/bh1750.h': combined+='volatile BH1750Status bh1750;\n'
combined+=r'''
static uint16_t channels[8]={1500,1500,1000,1500,1000,1000,1000,1000};
static void send_rc(int bad) {
    int j; memset(mock_packet,0,32);
    mock_packet[0]=0xAA; mock_packet[1]=0xAF; mock_packet[2]=3; mock_packet[3]=27;
    for(j=0;j<8;++j) { mock_packet[4+2*j]=channels[j]>>8; mock_packet[5+2*j]=channels[j]; }
    mock_packet[31]=LightLink_Checksum(mock_packet);
    if(bad) ++mock_packet[31];
    packet_available=1; RC_Analy();
}
static void step(int valid) {
    SysTick_count+=2; if(valid)send_rc(0);
    AutoFlight_Tick(0.006f);
    FlightPidControl(0.003f); MotorControl();
    FlightPidControl(0.003f); MotorControl();
    assert(range_reads==0 && !ALL_flag.height_lock && !ALL_flag.flow_control);
}
static void steps(int n) { int j; for(j=0;j<n;++j)step(1); }
static void toggle(int index) { channels[index]=channels[index]>1500?1000:2000; send_rc(0); }
static void launch(void) { steps(20); toggle(6); steps(3); assert(ALL_flag.unlock && AutoFlight_Active()); }
static void held_throttle(void) { launch(); steps(510); assert(auto_flight.state==AUTO_THR_HOLD); }
static void motor_zero(void) { assert(!TIM2->CCR1 && !TIM2->CCR2 && !TIM2->CCR3 && !TIM2->CCR4); }
int main(int argc,char **argv) {
    int j; float prev, held;
    const char *name=argv[1];
    AutoFlight_Init(); LightTask_Init();
    if(!strcmp(name,"startup_high_no_arm")) {
        channels[5]=channels[6]=channels[7]=2000; steps(100); assert(!ALL_flag.unlock); motor_zero();
    } else if(!strcmp(name,"no_range_required")) {
        launch(); assert(!flow_height_valid && flow_valid_samples==0);
        steps(510); assert(auto_flight.state==AUTO_THR_HOLD && TIM2->CCR1==500);
    } else if(!strcmp(name,"prearmed_zero_start")) {
        steps(20); ALL_flag.unlock=1; toggle(6); steps(3); assert(auto_flight.state==AUTO_SPOOL);
    } else if(!strcmp(name,"ramp_limited")) {
        launch(); prev=auto_flight.throttle;
        for(j=0;j<600;++j) { step(1); assert(auto_flight.throttle>=prev && auto_flight.throttle<=500.01f); assert(auto_flight.throttle-prev<1.1f); prev=auto_flight.throttle; }
    } else if(!strcmp(name,"reject_throttle")) {
        steps(20); channels[2]=1600; toggle(6); steps(2);
        assert(!ALL_flag.unlock && auto_flight.error==AUTO_REJECT_STICKS); motor_zero();
    } else if(!strcmp(name,"reject_tilt")) {
        steps(20); Angle.roll=15; toggle(6); steps(2);
        assert(!ALL_flag.unlock && auto_flight.error==AUTO_REJECT_TILT); motor_zero();
    } else if(!strcmp(name,"stop_immediate_on_frame")) {
        held_throttle(); assert(TIM2->CCR1>0); toggle(5);
        assert(!ALL_flag.unlock && auto_flight.state==AUTO_IDLE); motor_zero();
    } else if(!strcmp(name,"stop_wins_simultaneous_start")) {
        steps(20); channels[5]=channels[6]=2000; send_rc(0); steps(10);
        assert(!ALL_flag.unlock); motor_zero();
    } else if(!strcmp(name,"stop_latched_no_restart")) {
        held_throttle(); toggle(5); steps(1000); assert(!ALL_flag.unlock); motor_zero();
    } else if(!strcmp(name,"fresh_press_after_stop")) {
        held_throttle(); toggle(5); steps(50); toggle(6); steps(3);
        assert(ALL_flag.unlock && auto_flight.state==AUTO_SPOOL);
    } else if(!strcmp(name,"takeover_requires_match")) {
        held_throttle(); toggle(7); steps(60); held=auto_flight.throttle;
        assert(auto_flight.state==AUTO_TAKEOVER && auto_flight.takeover_pending && held==500);
        assert(ALL_flag.unlock && TIM2->CCR1==500);
        channels[2]=1667; steps(45);
        assert(auto_flight.state==AUTO_MANUAL && !auto_flight.takeover_pending);
        assert(fabsf((float)TIM2->CCR1-held)<=35.0f);
        channels[0]=1800; send_rc(0); assert(pidRoll.desired!=0);
        toggle(5); motor_zero(); assert(!ALL_flag.unlock);
    } else if(!strcmp(name,"early_takeover_match")) {
        launch(); steps(30); held=auto_flight.throttle;
        toggle(7); step(1); assert(auto_flight.state==AUTO_TAKEOVER);
        channels[2]=(uint16_t)(1000+held/2.45f); steps(45);
        assert(auto_flight.state==AUTO_MANUAL);
    } else if(!strcmp(name,"lost_rc_stops")) {
        held_throttle(); for(j=0;j<85;++j) step(0);
        assert(!ALL_flag.unlock && auto_flight.state==AUTO_RC_FAULT); motor_zero();
    } else if(!strcmp(name,"bad_packets_no_keepalive")) {
        held_throttle(); for(j=0;j<85;++j) { send_rc(1); step(0); }
        assert(!ALL_flag.unlock); motor_zero();
    } else if(!strcmp(name,"tilt_stops")) {
        held_throttle(); Angle.roll=55; step(1); assert(!ALL_flag.unlock); motor_zero();
    } else if(!strcmp(name,"emergency_stick")) {
        held_throttle(); channels[3]=1000; send_rc(0); motor_zero(); assert(!ALL_flag.unlock);
    } else if(!strcmp(name,"repeat_three_no_restart")) {
        held_throttle(); toggle(6); step(1);
        assert(auto_flight.state==AUTO_THR_HOLD && auto_flight.throttle==500);
    } else if(!strcmp(name,"aux1_cannot_enable_range_mode")) {
        held_throttle(); toggle(4); steps(100); assert(!ALL_flag.height_lock && !ALL_flag.flow_control);
    } else if(!strcmp(name,"legacy_sticks_cannot_unlock")) {
        steps(20); channels[2]=1900; steps(20); channels[2]=1000; steps(20);
        assert(!ALL_flag.unlock); motor_zero();
    } else if(!strcmp(name,"invalid_channel_rejected")) {
        steps(20); channels[0]=800; toggle(6); steps(20); assert(!ALL_flag.unlock); motor_zero();
    } else assert(0);
    puts("passed"); return 0;
}
'''
c=OUT/'actual_open_takeoff.c'; c.write_text(combined,encoding='utf-8')
exe=OUT/'actual_open_takeoff.exe'
command=[str(CC),'-std=c99','-O1','-Wall','-Wextra','-Wno-unused-parameter',
         '-I'+str(FLIGHT/'USER'),'-I'+str(FLIGHT/'HAL'),str(c),'-o',str(exe),'-lm']
r=subprocess.run(command,env=env,capture_output=True,text=True,errors='replace')
(OUT/'compile_log.txt').write_text(r.stdout+r.stderr,encoding='utf-8')
if r.returncode: print(r.stdout+r.stderr); raise SystemExit(r.returncode)
cases=re.findall(r'!strcmp\(name,"([^"]+)"\)',combined)
results=[]
for case in cases:
    r=subprocess.run([str(exe),case],env=env,capture_output=True,text=True,errors='replace',timeout=15)
    results.append({'case':case,'returncode':r.returncode,'diagnostics':r.stdout+r.stderr})
    print(case,'PASS' if not r.returncode else 'FAIL',r.stderr.strip())
feedback=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
static int beep_pin;
volatile uint32_t SysTick_count;
#define BEEP_H beep_pin=1
#define BEEP_L beep_pin=0
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t m) {}
static uint8_t NRF24L01_RxPacket(uint8_t *p) { return 1; }
static void RC_Analy(void) {}
'''+source(REMOTE,'HAL/light_link_protocol.h')+source(REMOTE,'HAL/light_feedback.h')+source(REMOTE,'HAL/light_feedback.c')+r'''
int main(int argc,char **argv) {
    int i,prev=0,rises=0; uint8_t p[32]; LightLinkStatus s={0};
    RemoteFeedback_Init(); s.diagnostic_valid=1; s.auto_state=3;
    LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,0));
    if(!strcmp(argv[1],"takeover_wait_notice")) {
        for(i=0;i<3000;++i) {
            if(i%20==0) assert(RemoteFeedback_Accept(p,i));
            RemoteFeedback_BuzzerTick(i);
            if(beep_pin && !prev)++rises; prev=beep_pin;
        }
        assert(rises==5);
    } else if(!strcmp(argv[1],"notice_expiry_keeps_key_click")) {
        RemoteFeedback_BuzzerTick(600); assert(!beep_pin);
        RemoteFeedback_KeyClick(600); RemoteFeedback_BuzzerTick(600); assert(beep_pin);
        RemoteFeedback_BuzzerTick(631); assert(!beep_pin);
    } else if(!strcmp(argv[1],"manual_feedback_ends_notice")) {
        s.auto_state=4; LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,100));
        RemoteFeedback_BuzzerTick(700); assert(!beep_pin);
    } else if(!strcmp(argv[1],"stop_feedback_ends_notice")) {
        s.auto_state=0; LightLink_Encode(p,&s); assert(RemoteFeedback_Accept(p,100));
        RemoteFeedback_BuzzerTick(700); assert(!beep_pin);
    } else if(!strcmp(argv[1],"rejection_notice")) {
        s.auto_state=0; s.auto_error=2; s.start_request_id=1;
        LightLink_Encode(p,&s);
        for(i=0;i<1500;++i) {
            if(i%20==0) assert(RemoteFeedback_Accept(p,i));
            RemoteFeedback_BuzzerTick(i);
            if(beep_pin && !prev)++rises; prev=beep_pin;
        }
        assert(rises==2);
    } else if(!strcmp(argv[1],"malformed_feedback")) {
        p[31]++; assert(!RemoteFeedback_Accept(p,0));
    } else assert(0);
    puts("passed"); return 0;
}
'''
remote_c=OUT/'remote_feedback.c'; remote_c.write_text(feedback,encoding='utf-8')
remote_exe=OUT/'remote_feedback.exe'
r=subprocess.run([str(CC),'-std=c99','-O1',str(remote_c),'-o',str(remote_exe)],env=env,capture_output=True,text=True,errors='replace')
if r.returncode: print(r.stdout+r.stderr); raise SystemExit(r.returncode)
for case in re.findall(r'!strcmp\(argv\[1\],"([^"]+)"\)',feedback):
    r=subprocess.run([str(remote_exe),case],env=env,capture_output=True,text=True,errors='replace',timeout=15)
    results.append({'case':case,'returncode':r.returncode,'diagnostics':r.stdout+r.stderr})
    print(case,'PASS' if not r.returncode else 'FAIL',r.stderr.strip())
scheduler=read(FLIGHT/'USER/scheduler.c')
assert not any(n in scheduler for n in ('flow_get_data(', 'flow_data_sins(', 'HeightPidControl(', 'FlowPidControl('))
old=json.loads((HERE/'原版本哈希.json').read_text(encoding='utf-8'))
for board,base in [('飞控',ROOT/'02_飞控源码与说明/02主控源码'),('遥控器',ROOT/'02_飞控源码与说明/03遥控源码')]:
    for name,digest in old[board].items(): assert hashlib.sha256((base/name).read_bytes()).hexdigest()==digest, name
report={'scope':'Actual C with hardware mocks; no flight, lift or altitude proof',
        'passed':all(x['returncode']==0 for x in results),'case_count':len(results),
        'original_versions_unchanged':True,'scheduler_has_no_range_or_position_calls':True,
        'cases':results,'source_sha256':{n:hashlib.sha256((FLIGHT/n).read_bytes()).hexdigest() for n in names}}
(OUT/'test_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
raise SystemExit(0 if report['passed'] else 1)
