"""Exercise the unchanged supplier flow_get_data body with a mocked I2C bus.

This is a host-side reproduction of parsing and stale-data behavior, not an
STM32 build or a flight test. It never accesses attached hardware.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import sys

sys.stdout.reconfigure(encoding="utf-8")
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--cc", default=r"C:\mingw64\bin\gcc.exe")
args = parser.parse_args()

workspace = Path(__file__).resolve().parents[1]
source = workspace / "02_飞控源码与说明" / "02主控源码" / "HARDWARE" / "flow.c"
raw = source.read_bytes()
try:
    text = raw.decode("utf-8-sig")
except UnicodeDecodeError:
    text = raw.decode("gb18030")
body = text[text.index("void flow_get_data(void)"):]
body = body.replace("\r\n", "\n")

prefix = r'''
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
'''
suffix = r'''
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
'''
test_source = Path(__file__).with_name("flow_reader_host_test.c")
test_binary = Path(__file__).with_name("flow_reader_host_test.exe")
test_source.write_text(prefix + body + suffix, encoding="utf-8", newline="\n")
compiler = Path(args.cc)
compiler_env = os.environ.copy()
compiler_env["PATH"] = str(compiler.parent) + os.pathsep + compiler_env.get("PATH", "")
try:
    build = subprocess.run(
        [str(compiler), "-std=c99", "-Wall", "-Wextra", "-Werror",
         test_source.name, "-o", test_binary.name],
        cwd=test_source.parent, env=compiler_env,
        capture_output=True, text=True, encoding="utf-8", errors="replace",
    )
except OSError as error:
    result = {
        "status": "not_run",
        "source": str(source),
        "source_sha256": hashlib.sha256(raw).hexdigest(),
        "compiler": str(compiler),
        "error": str(error),
        "winerror": getattr(error, "winerror", None),
        "scope": "Compiler could not start; no C test or STM32 build passed",
    }
    Path(__file__).with_name("flow_reader_host_test_result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("C test not run: compiler startup failed (Windows error "
          + str(result["winerror"]) + ").")
    raise SystemExit(2)
if build.returncode:
    result = {"status": "build_failed", "compiler": str(compiler),
              "build_returncode": build.returncode, "stdout": build.stdout,
              "stderr": build.stderr, "source_sha256": hashlib.sha256(raw).hexdigest()}
    Path(__file__).with_name("flow_reader_host_test_result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    raise RuntimeError(f"Compiler exit {build.returncode}: {build.stdout}{build.stderr}")
run = subprocess.run([str(test_binary)], capture_output=True, text=True, env=compiler_env)
result = {
    "status": "passed" if run.returncode == 0 else "run_failed",
    "source": str(source),
    "source_sha256": hashlib.sha256(raw).hexdigest(),
    "compiler": str(compiler),
    "build_returncode": build.returncode,
    "run_returncode": run.returncode,
    "output": run.stdout,
    "environment_fix": "Compiler bin prepended to child-process PATH; ASCII relative build arguments",
    "scope": "Unchanged supplier function with mocked I2C; not an STM32 build or flight test",
}
Path(__file__).with_name("flow_reader_host_test_result.json").write_text(
    json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
)
print(run.stdout)
if run.returncode:
    raise RuntimeError(run.stderr)
