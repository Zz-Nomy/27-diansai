"""One-time integration record. Sources are already integrated; do not reapply.

Compiler compatibility and remote I2C fixes are also present in the delivered
working tree; the complete original transmitter is preserved separately.
"""
from pathlib import Path
import hashlib
import json
import shutil

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FLIGHT = ROOT / '02_飞控源码与说明' / '02主控源码'
REMOTE = ROOT / '02_飞控源码与说明' / '03遥控源码'
if b'IIC_Send_Command' in (FLIGHT/'CONFIG/I2C.h').read_bytes():
    raise SystemExit('Already integrated. Use build_arm.py or verify_light_task.py.')

def edit(base, name, transform):
    path = base / name
    data = path.read_bytes()
    try:
        text = data.decode('utf-8'); enc = 'utf-8'
    except UnicodeDecodeError:
        text = data.decode('gbk'); enc = 'gbk'
    newline = '\r\n' if b'\r\n' in data else '\n'
    text = text.replace('\r\n', '\n')
    updated = transform(text)
    assert updated != text, name + ': no change'
    if base == FLIGHT:
        backup = HERE / '原始源码' / name
        if not backup.exists():
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, backup)
    path.write_bytes(updated.replace('\n', newline).encode(enc))

def one(text, before, after):
    assert text.count(before) == 1, repr(before)
    return text.replace(before, after, 1)

edit(FLIGHT, 'CONFIG/I2C.h', lambda s: one(s,
    'extern int8_t IIC_read_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len);',
    '''extern int8_t IIC_read_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len);
/* Registerless devices. addr is an even 8-bit WRITE address (7-bit << 1).
 * Return 0 on success, 1 on failure, as in the existing bus driver. */
extern int8_t IIC_Send_Command(uint8_t addr, uint8_t command);
extern int8_t IIC_Receive_Bytes(uint8_t addr, uint8_t *data, uint8_t len);'''))
edit(FLIGHT, 'CONFIG/IIC_SOFTWARE.c', lambda s: s + '''

/* Raw transactions for BH1750: never send a fictitious register address. */
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
''')
edit(FLIGHT, 'USER/INIT.c', lambda s: one(one(s,
    '#include "auto_flight.h"', '#include "auto_flight.h"\n#include "bh1750.h"\n#include "light_task.h"'),
    '\tAutoFlight_Init();', '\tAutoFlight_Init();\n\tBH1750_Init();\n\tLightTask_Init();'))
edit(FLIGHT, 'USER/scheduler.c', lambda s: one(one(s,
    '#include "auto_flight.h"', '#include "auto_flight.h"\n#include "bh1750.h"\n#include "light_task.h"'),
    '            AutoFlight_Tick(0.006f);\n            HeightPidControl(0.006f);',
    '''            BH1750_Tick(0.006f);
            AutoFlight_Tick(0.006f);
            LightTask_Tick(0.006f);
            HeightPidControl(0.006f);
            PilotLED();'''))
edit(FLIGHT, 'USER/main.c', lambda s: one(s,
    '\t\t\tPilotLED();', '\t\t\t/* LED refresh runs in the 6 ms scheduler. */'))

def fix_delay(s):
    s = one(s, 'static volatile uint32_t usTicks = 0;',
        '/* Only TIM1 updates SysTick_count (one tick = 3 ms). */\nstatic volatile uint32_t systick_ms = 0;')
    s = one(s, '\tSysTick_count++;', '\t++systick_ms;')
    start = s.index('    return (SysTick_count*1000')
    end = s.index('\n}', start)
    s = s[:start] + '''    uint32_t ms, value, reload = SysTick->LOAD + 1U;
    do { ms = systick_ms; value = SysTick->VAL; } while (ms != systick_ms);
    return ms * 1000U + (reload - value) / (SystemCoreClock / 1000000U);''' + s[end:]
    s = one(s, 'uint32_t last_systick;\n', '')
    s = one(s, '\tlast_systick=SysTick_count;\n\twhile(SysTick_count - last_systick < nms);',
        '\tuint32_t start_ms = systick_ms;\n\twhile ((uint32_t)(systick_ms - start_ms) < nms);')
    return s
edit(FLIGHT, 'USER/delay.c', fix_delay)

def fix_auto(s):
    s = one(s, '#include "remote.h"', '#include "remote.h"\n#include "light_task.h"')
    s = one(s, 'return auto_flight.state == AUTO_HOVER;',
        'return auto_flight.state == AUTO_HOVER && !LightTask_RemoteLocked();')
    s = one(s, 'if (aux1 != last_aux1 && AutoFlight_Active())',
        'if (aux1 != last_aux1 && AutoFlight_Active() && !LightTask_RemoteLocked())')
    s = one(s, 'if (auto_flight.takeover_pending && Remote.thr > 1150U &&',
        'if ((!LightTask_RemoteLocked() || auto_flight.state != AUTO_HOVER) &&\n        auto_flight.takeover_pending && Remote.thr > 1150U &&')
    start = s.index('    if (Remote.thr >= 1250U && Remote.thr <= 1750U)')
    end = s.index('    if (auto_flight.state == AUTO_SPOOL)', start)
    block = s[start:end]
    s = s[:start] + '''    if (LightTask_RemoteLocked()) {
        landing_arm_seconds = landing_gesture_seconds = 0.0f;
        landing_enabled = 0;
        auto_flight.takeover_pending = 0;
    } else {
''' + block + '    }\n' + s[end:]
    return s
edit(FLIGHT, 'HAL/auto_flight.c', fix_auto)

def fix_control(s):
    before = 'void FlowPidControl(float dt)'
    s = one(s, before, '''static uint8_t flow_capture_requested;
static float flow_hold_x, flow_hold_y;
void FlowControl_LatchPosition(void)
{
    flow_hold_x = flow_x_lpf_att_i;
    flow_hold_y = flow_y_lpf_att_i;
    flow_capture_requested = 1;
}

''' + before)
    before = '\t\tcase PROCESS_31://进入\n\t\t\t{'
    after = before + '''
                if (flow_capture_requested) {
                    flow_capture_requested = 0;
                    set_pos = 0;
                    pidRest(&pPidObject[8], 4);
                    pidPositionX.desired = flow_hold_x;
                    pidPositionY.desired = flow_hold_y;
                }
'''
    return one(s, before, after)
edit(FLIGHT, 'HAL/control.c', fix_control)
edit(FLIGHT, 'HAL/control.h', lambda s: one(s,
    'extern void FlowPidControl(float dt);',
    'extern void FlowPidControl(float dt);\nextern void FlowControl_LatchPosition(void);'))

def fix_receiver(s):
    s = one(s, '#include "auto_flight.h"',
        '#include "auto_flight.h"\n#include "light_task.h"\n#include "light_link.h"')
    s = one(s, 'if(Remote.yaw>1820)', 'if(!LightTask_RemoteLocked() && Remote.yaw>1820)')
    s = one(s, 'else if(Remote.yaw <1180)',
        'else if(!LightTask_RemoteLocked() && Remote.yaw <1180)')
    s = one(s, '\t\t\t\tremote_unlock();',
        '\t\t\t\tremote_unlock();\n                LightLink_OnControlFrame(RC_rxData);')
    return s
edit(FLIGHT, 'HAL/remote.c', fix_receiver)

def fix_led(s):
    s = one(s, '#include "auto_flight.h"', '#include "auto_flight.h"\n#include "light_task.h"')
    before = '    if (alarm_was_active) {'
    s = one(s, before, '''    if (LightTask_AlertActive()) {
        auto_alarm_leds(LightTask_AlertLedOn() ?
                        (AUTO_BLUE_LED_MASK | AUTO_RED_LED_MASK) : 0U);
        alarm_was_active = 1;
        return;
    }
''' + before)
    return one(s, 'if(SysTick_count - LastTime < LED.FlashTime)',
        'if((uint32_t)(SysTick_count - LastTime) * 3U < LED.FlashTime)')
edit(FLIGHT, 'HARDWARE/LED.c', fix_led)

def fix_radio(s):
    s = one(s, '#include <string.h>', '#include <string.h>\n#include "delay.h"')
    s = one(s, '\tu8 state;   \n\tClr_NRF24L01_CE;',
        '\tu8 state;\n    uint32_t start_us = GetSysTime_us();\n\tClr_NRF24L01_CE;\n    NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, 0x70);\n    NRF24L01_Write_Reg(FLUSH_TX, 0xFF);')
    start = s.index('\twhile(READ_NRF24L01_IRQ!=0);')
    end = s.index('\n', start)
    s = s[:start] + '''    while (READ_NRF24L01_IRQ != 0) {
        if ((uint32_t)(GetSysTime_us() - start_us) >= 2500U) {
            Clr_NRF24L01_CE;
            NRF24L01_Write_Reg(FLUSH_TX, 0xFF);
            return 0xFF;
        }
    }
    Clr_NRF24L01_CE;''' + s[end:]
    return s
edit(FLIGHT, 'HARDWARE/nrf24l01.c', lambda s: fix_radio(s) + '''

/* A requested 32-byte status reply, using ordinary half-duplex NRF24L01.
 * Called from the foreground only. Always return to RX, including timeout. */
uint8_t NRF24L01_SendReply(uint8_t *payload)
{
    uint8_t result;
    uint32_t start_us = GetSysTime_us();
    /* Allow the transmitter to finish its uplink and settle into RX. */
    while ((uint32_t)(GetSysTime_us() - start_us) < 400U) {}
    Clr_NRF24L01_CE;
    NRF24L01_Write_Buf(SPI_WRITE_REG + TX_ADDR, (u8 *)TX_ADDRESS, TX_ADR_WIDTH);
    NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0E);
    result = NRF24L01_TxPacket(payload);
    RX_Mode();
    return result;
}
''')
edit(FLIGHT, 'HARDWARE/nrf24l01.h', lambda s: one(s,
    'extern void NRF24L01_init(void);',
    'extern void NRF24L01_init(void);\nuint8_t NRF24L01_SendReply(uint8_t *payload);'))

def add_project_files(s, anchor, names):
    files = ''.join('''            <File>
              <FileName>{filename}</FileName>
              <FileType>1</FileType>
              <FilePath>.\\{name}</FilePath>
            </File>
'''.format(filename=Path(n).name, name=n.replace('/', '\\')) for n in names)
    return one(s, anchor, files + anchor)
edit(FLIGHT, 'mini.uvprojx', lambda s: add_project_files(s,
    '            <File>\n              <FileName>flow.c</FileName>',
    ['HARDWARE/bh1750.c','HAL/light_task.c','HAL/light_link.c']))

# The complete extracted vendor remote project is preserved under this run's
# archive directory. These edits affect only the new working copy.
shutil.copy2(FLIGHT/'HAL/light_link_protocol.h', REMOTE/'HAL/light_link_protocol.h')
edit(REMOTE, 'HARDWARE/nrf24l01.c', lambda s: fix_radio(s) + '''

void NRF24L01_Listen(void)
{
    Clr_NRF24L01_CE;
    NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0F);
    Set_NRF24L01_CE;
}
''')
edit(REMOTE, 'HARDWARE/nrf24l01.h', lambda s: one(s,
    'extern void NRF24L01_init(void);',
    'extern void NRF24L01_init(void);\nvoid NRF24L01_Listen(void);'))
edit(REMOTE, 'USER/sys.h', lambda s: one(s,
    'extern uint32_t SysTick_count;', 'extern volatile uint32_t SysTick_count;'))

def fix_remote_sys(s):
    s = one(s, '#include "remote.h"', '#include "remote.h"\n#include "light_feedback.h"')
    s = one(s, 'uint32_t SysTick_count = 0;', 'volatile uint32_t SysTick_count = 0;')
    s = one(s, '\tRC_INIT();', '\tRemoteFeedback_Init();\n\tRC_INIT();')
    start = s.index('    \n\tstatic uint8_t Count_10ms;') if '    \n\tstatic uint8_t Count_10ms;' in s else s.index('\tstatic uint8_t Count_10ms;')
    end = s.index('\n}', start)
    s = s[:start] + '\n    ++SysTick_count;\n    RemoteFeedback_BuzzerTick(SysTick_count);\n' + s[end:]
    return s
edit(REMOTE, 'USER/sys.c', fix_remote_sys)
edit(REMOTE, 'USER/main.c', lambda s: one(one(s,
    '#include "usart.h"', '#include "usart.h"\n#include "light_feedback.h"'),
    '\t\t//所有数据都在10ms中断处理，详情请见SYS.C', '\t\tRemoteControl_Poll();'))

def fix_remote_rc(s):
    s = one(s, '#include "delay.h"', '#include "delay.h"\n#include "light_feedback.h"')
    start = s.index('\tfor(uint8_t i=4;i<26;i+=2)')
    end = s.index('\n}', start)
    s = s[:start] + '''    uint8_t i;
    uint16_t channels[8];
    channels[0] = Remote.roll; channels[1] = Remote.pitch;
    channels[2] = Remote.thr; channels[3] = Remote.yaw;
    channels[4] = Remote.AUX1; channels[5] = Remote.AUX2;
    channels[6] = Remote.AUX3; channels[7] = Remote.AUX4;
    memset(tx_data + 4, 0, 28);
    for (i = 0; i < 8U; ++i) {
        tx_data[4U + i * 2U] = (uint8_t)(channels[i] >> 8);
        tx_data[5U + i * 2U] = (uint8_t)channels[i];
    }
    tx_data[20] = 'L'; tx_data[21] = 'T';
    tx_data[22] = 1; tx_data[23] = 0xA5;
    tx_data[31] = LightLink_Checksum(tx_data);
    TX_Mode();
    NRF24L01_TxPacket(tx_data);
    NRF24L01_Listen();''' + s[end:]
    s = one(s, '#include <math.h>', '#include <math.h>\n#include <string.h>')
    s = one(s, '\t\t\t\tBEEP_L;', '\t\t\t\t/* Key click expiry is handled by the 1 ms buzzer task. */')
    s = one(s, '\t\t\tBEEP_H;', '\t\t\tRemoteFeedback_KeyClick(SysTick_count);')
    return s
edit(REMOTE, 'HAL/remote.c', fix_remote_rc)
edit(REMOTE, 'remote.uvprojx', lambda s: add_project_files(s,
    '            <File>\n              <FileName>remote.c</FileName>', ['HAL/light_feedback.c']))

manifest = {str(p.relative_to(HERE/'原始源码')): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (HERE/'原始源码').rglob('*') if p.is_file()}
(HERE/'backup_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
print('Source integration applied. Original encodings and backups preserved.')
