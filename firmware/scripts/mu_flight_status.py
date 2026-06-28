#!/usr/bin/env python3
import re
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OPENOCD = Path.home() / ".platformio/packages/tool-openocd/bin/openocd"
NM = Path.home() / ".platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm"

FIELDS = [
    "magic",
    "version",
    "system_hz",
    "uptime_ms",
    "loop_count",
    "sample_count",
    "latest_raw",
    "latest_mv",
    "baseline_raw",
    "baseline_mv",
    "state",
    "flash_ok",
    "flash_jedec",
    "logger_ok",
    "logger_status",
    "run_id",
    "records_written",
    "event_count",
    "pressure_count",
    "boot_logged",
    "event_log_failures",
    "pressure_log_failures",
    "baro_ok",
    "pressure_pa",
    "temperature_centi_c",
    "altitude_mm",
    "last_pressure_ms",
    "last_event_ms",
    "last_event_baseline_mv",
    "last_event_min_mv",
    "last_event_amplitude_mv",
    "dma_half_count",
    "dma_full_count",
    "dma_error_count",
    "adc_overrun_count",
    "event_queue_depth",
    "event_queue_max_depth",
    "event_queue_drops",
]


def symbol_address(env_name):
    elf = ROOT / f".pio/build/{env_name}/firmware.elf"
    proc = subprocess.run(
        [str(NM), "-S", str(elf)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    )
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[3] == "g_mu_flight":
            return int(parts[0], 16)
    raise RuntimeError("g_mu_flight symbol not found")


def parse_words(text):
    words = []
    for line in text.splitlines():
        if not line.startswith("0x"):
            continue
        hex_words = re.findall(r"\b[0-9a-fA-F]{8}\b", line)
        if hex_words and line.startswith("0x" + hex_words[0]):
            hex_words = hex_words[1:]
        words.extend(int(word, 16) for word in hex_words)
    return words


def signed32(value):
    return value - 0x100000000 if value & 0x80000000 else value


def read_status(address):
    proc = subprocess.run(
        [
            str(OPENOCD),
            "-f",
            "interface/stlink.cfg",
            "-f",
            "target/stm32f0x.cfg",
            "-c",
            "init",
            "-c",
            f"mdw 0x{address:08x} {len(FIELDS)}",
            "-c",
            "shutdown",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        timeout=12,
    )
    words = parse_words(proc.stdout + proc.stderr)
    if proc.returncode != 0 or len(words) < len(FIELDS):
        return None
    data = dict(zip(FIELDS, words[: len(FIELDS)]))
    for name in ("pressure_pa", "temperature_centi_c", "altitude_mm"):
        data[name] = signed32(data[name])
    return data


def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    period_s = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    env_name = sys.argv[3] if len(sys.argv) > 3 else "mu_f042_flight_logger"
    address = symbol_address(env_name)
    print(f"env={env_name} g_mu_flight=0x{address:08x}", flush=True)
    t0 = time.time()
    for _ in range(count):
        data = read_status(address)
        elapsed = time.time() - t0
        if data is None:
            print(f"t={elapsed:6.1f}s read_failed", flush=True)
        else:
            print(
                "t={:6.1f}s up={}ms state={} latest={}mV base={}mV "
                "flash={} jedec=0x{:06X} logger={} lstat={} run={} rec={} "
                "evt={} prs={} baro={} p={}Pa temp={:.2f}C alt={}mm "
                "last_amp={}mV q={}/{} qdrop={} dma={}/{} derr={} aovr={} "
                "fail_e={} fail_p={}".format(
                    elapsed,
                    data["uptime_ms"],
                    data["state"],
                    data["latest_mv"],
                    data["baseline_mv"],
                    data["flash_ok"],
                    data["flash_jedec"],
                    data["logger_ok"],
                    data["logger_status"],
                    data["run_id"],
                    data["records_written"],
                    data["event_count"],
                    data["pressure_count"],
                    data["baro_ok"],
                    data["pressure_pa"],
                    data["temperature_centi_c"] / 100.0,
                    data["altitude_mm"],
                    data["last_event_amplitude_mv"],
                    data["event_queue_depth"],
                    data["event_queue_max_depth"],
                    data["event_queue_drops"],
                    data["dma_half_count"],
                    data["dma_full_count"],
                    data["dma_error_count"],
                    data["adc_overrun_count"],
                    data["event_log_failures"],
                    data["pressure_log_failures"],
                ),
                flush=True,
            )
        time.sleep(period_s)


if __name__ == "__main__":
    main()
