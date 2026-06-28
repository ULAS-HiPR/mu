#!/usr/bin/env python3
import csv
import datetime as dt
import re
import struct
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ELF = ROOT / ".pio/build/mu_f042_pulse_capture/firmware.elf"
OPENOCD = Path.home() / ".platformio/packages/tool-openocd/bin/openocd"
NM = Path.home() / ".platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm"
OUT_DIR = ROOT / "captures"

FIELDS = [
    "magic",
    "version",
    "system_hz",
    "sample_interval_us",
    "pre_samples",
    "post_samples",
    "total_samples",
    "command",
    "state",
    "ready",
    "event_seq",
    "loop_count",
    "sample_count",
    "latest_raw",
    "latest_mv",
    "baseline_raw",
    "baseline_mv",
    "trigger_delta_raw",
    "trigger_raw",
    "trigger_mv",
    "trigger_baseline_raw",
    "trigger_baseline_mv",
    "trigger_sample_count",
    "min_raw",
    "max_raw",
]


def symbol_addresses():
    proc = subprocess.run(
        [str(NM), "-S", str(ELF)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    )
    symbols = {}
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[3] in {"g_mu_pulse", "g_mu_pulse_samples"}:
            symbols[parts[3]] = int(parts[0], 16)
    missing = {"g_mu_pulse", "g_mu_pulse_samples"} - set(symbols)
    if missing:
        raise RuntimeError(f"missing symbols: {sorted(missing)}")
    return symbols


def run_openocd(commands):
    cmd = [
        str(OPENOCD),
        "-f",
        "interface/stlink.cfg",
        "-f",
        "target/stm32f0x.cfg",
        "-c",
        "init",
    ]
    for command in commands:
        cmd.extend(["-c", command])
    cmd.extend(["-c", "shutdown"])
    return subprocess.run(
        cmd,
        cwd=ROOT,
        text=True,
        capture_output=True,
        timeout=12,
    )


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


def read_header(header_addr):
    proc = run_openocd([f"mdw 0x{header_addr:08x} {len(FIELDS)}"])
    words = parse_words(proc.stdout + proc.stderr)
    if proc.returncode != 0 or len(words) < len(FIELDS):
        return None
    return dict(zip(FIELDS, words[: len(FIELDS)]))


def read_samples(sample_addr, sample_count):
    word_count = (sample_count + 1) // 2
    proc = run_openocd([f"mdw 0x{sample_addr:08x} {word_count}"])
    words = parse_words(proc.stdout + proc.stderr)
    if proc.returncode != 0 or len(words) < word_count:
        raise RuntimeError("failed to read capture samples")

    samples = []
    for word in words[:word_count]:
        samples.append(word & 0xFFFF)
        samples.append((word >> 16) & 0xFFFF)
    return samples[:sample_count]


def write_command(header_addr, command):
    command_addr = header_addr + FIELDS.index("command") * 4
    proc = run_openocd([f"mww 0x{command_addr:08x} {command}"])
    if proc.returncode != 0:
        raise RuntimeError("failed to write rearm command")


def raw_to_mv(raw):
    return raw * 3300.0 / 4095.0


def save_capture(header, samples):
    OUT_DIR.mkdir(exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    seq = header["event_seq"]
    csv_path = OUT_DIR / f"mu_pulse_{seq:04d}_{stamp}.csv"
    png_path = OUT_DIR / f"mu_pulse_{seq:04d}_{stamp}.png"
    svg_path = OUT_DIR / f"mu_pulse_{seq:04d}_{stamp}.svg"
    sample_interval_us = header["sample_interval_us"]
    pre = header["pre_samples"]

    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sample", "time_us", "raw", "mv"])
        for i, raw in enumerate(samples):
            writer.writerow([i, (i - pre + 1) * sample_interval_us, raw, f"{raw_to_mv(raw):.3f}"])

    try:
        import matplotlib.pyplot as plt

        times = [(i - pre + 1) * sample_interval_us for i in range(len(samples))]
        millivolts = [raw_to_mv(raw) for raw in samples]
        baseline_mv = header["trigger_baseline_mv"]
        trigger_mv = header["trigger_mv"]

        fig, ax = plt.subplots(figsize=(10, 4.8))
        ax.plot(times, millivolts, linewidth=1.2)
        ax.axhline(baseline_mv, color="tab:green", linestyle="--", linewidth=1.0, label="baseline")
        ax.axhline(trigger_mv, color="tab:red", linestyle=":", linewidth=1.0, label="trigger sample")
        ax.axvline(0, color="black", linestyle="-", linewidth=0.8, alpha=0.5)
        ax.set_title(f"Mu pulse capture seq {seq}")
        ax.set_xlabel("time from trigger (us)")
        ax.set_ylabel("CATCH_OUT (mV)")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
        fig.tight_layout()
        fig.savefig(png_path, dpi=160)
        plt.close(fig)
        svg_path = None
    except Exception as exc:
        print(f"plot_failed: {exc}", flush=True)
        png_path = None
        save_svg_plot(svg_path, header, samples)

    return csv_path, png_path, svg_path


def save_svg_plot(svg_path, header, samples):
    sample_interval_us = header["sample_interval_us"]
    pre = header["pre_samples"]
    width = 1000
    height = 480
    left = 70
    right = 24
    top = 36
    bottom = 58
    plot_w = width - left - right
    plot_h = height - top - bottom
    times = [(i - pre + 1) * sample_interval_us for i in range(len(samples))]
    values = [raw_to_mv(raw) for raw in samples]
    min_t = min(times)
    max_t = max(times)
    min_v = min(values + [header["trigger_mv"], header["trigger_baseline_mv"]])
    max_v = max(values + [header["trigger_mv"], header["trigger_baseline_mv"]])
    pad_v = max(50.0, (max_v - min_v) * 0.12)
    min_v -= pad_v
    max_v += pad_v

    def x_scale(t):
        return left + (t - min_t) * plot_w / (max_t - min_t)

    def y_scale(v):
        return top + (max_v - v) * plot_h / (max_v - min_v)

    points = " ".join(f"{x_scale(t):.2f},{y_scale(v):.2f}" for t, v in zip(times, values))
    baseline_y = y_scale(header["trigger_baseline_mv"])
    trigger_y = y_scale(header["trigger_mv"])
    zero_x = x_scale(0)
    title = f"Mu pulse capture seq {header['event_seq']}"

    x_ticks = [min_t, 0, max_t]
    y_ticks = [min_v, header["trigger_mv"], header["trigger_baseline_mv"], max_v]

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width / 2:.1f}" y="24" text-anchor="middle" font-family="sans-serif" font-size="18">{title}</text>',
        f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="#222" stroke-width="1"/>',
        f'<line x1="{left}" y1="{baseline_y:.2f}" x2="{left + plot_w}" y2="{baseline_y:.2f}" stroke="#2ca02c" stroke-dasharray="7 5" stroke-width="1.2"/>',
        f'<line x1="{left}" y1="{trigger_y:.2f}" x2="{left + plot_w}" y2="{trigger_y:.2f}" stroke="#d62728" stroke-dasharray="3 4" stroke-width="1.2"/>',
        f'<line x1="{zero_x:.2f}" y1="{top}" x2="{zero_x:.2f}" y2="{top + plot_h}" stroke="#111" stroke-width="1" opacity="0.55"/>',
        f'<polyline points="{points}" fill="none" stroke="#1f77b4" stroke-width="1.4"/>',
        f'<text x="{left + plot_w - 8}" y="{baseline_y - 6:.2f}" text-anchor="end" font-family="sans-serif" font-size="12" fill="#2ca02c">baseline {header["trigger_baseline_mv"]} mV</text>',
        f'<text x="{left + plot_w - 8}" y="{trigger_y - 6:.2f}" text-anchor="end" font-family="sans-serif" font-size="12" fill="#d62728">trigger {header["trigger_mv"]} mV</text>',
        f'<text x="{left + plot_w / 2:.1f}" y="{height - 14}" text-anchor="middle" font-family="sans-serif" font-size="13">time from trigger (us)</text>',
        f'<text x="18" y="{top + plot_h / 2:.1f}" transform="rotate(-90 18 {top + plot_h / 2:.1f})" text-anchor="middle" font-family="sans-serif" font-size="13">CATCH_OUT (mV)</text>',
    ]

    for tick in x_ticks:
        x = x_scale(tick)
        parts.append(f'<line x1="{x:.2f}" y1="{top + plot_h}" x2="{x:.2f}" y2="{top + plot_h + 5}" stroke="#222"/>')
        parts.append(f'<text x="{x:.2f}" y="{top + plot_h + 22}" text-anchor="middle" font-family="sans-serif" font-size="11">{tick:.0f}</text>')

    for tick in y_ticks:
        y = y_scale(tick)
        parts.append(f'<line x1="{left - 5}" y1="{y:.2f}" x2="{left}" y2="{y:.2f}" stroke="#222"/>')
        parts.append(f'<text x="{left - 8}" y="{y + 4:.2f}" text-anchor="end" font-family="sans-serif" font-size="11">{tick:.0f}</text>')

    parts.append("</svg>")
    svg_path.write_text("\n".join(parts), encoding="utf-8")


def main():
    wanted = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    timeout_s = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0
    poll_s = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    symbols = symbol_addresses()
    header_addr = symbols["g_mu_pulse"]
    sample_addr = symbols["g_mu_pulse_samples"]
    print(f"header=0x{header_addr:08x} samples=0x{sample_addr:08x}", flush=True)

    # Clear any stale capture after flashing/restarting.
    try:
        write_command(header_addr, 1)
    except RuntimeError as exc:
        print(f"initial_rearm_failed: {exc}", flush=True)

    saved = 0
    seen_seq = -1
    start = time.time()
    while saved < wanted and time.time() - start < timeout_s:
        header = read_header(header_addr)
        elapsed = time.time() - start
        if header is None:
            print(f"t={elapsed:6.1f}s read_failed", flush=True)
            time.sleep(poll_s)
            continue

        print(
            "t={:6.1f}s state={} ready={} seq={} latest={}mV base={}mV "
            "trig={}mV min_raw={} max_raw={} loop={}".format(
                elapsed,
                header["state"],
                header["ready"],
                header["event_seq"],
                header["latest_mv"],
                header["baseline_mv"],
                header["trigger_mv"],
                header["min_raw"],
                header["max_raw"],
                header["loop_count"],
            ),
            flush=True,
        )

        if header["ready"] == 1 and header["event_seq"] != seen_seq:
            samples = read_samples(sample_addr, header["total_samples"])
            csv_path, png_path, svg_path = save_capture(header, samples)
            print(f"saved seq={header['event_seq']} csv={csv_path}", flush=True)
            if png_path is not None:
                print(f"saved seq={header['event_seq']} png={png_path}", flush=True)
            if svg_path is not None:
                print(f"saved seq={header['event_seq']} svg={svg_path}", flush=True)
            seen_seq = header["event_seq"]
            saved += 1
            write_command(header_addr, 1)

        time.sleep(poll_s)

    print(f"done saved={saved}", flush=True)


if __name__ == "__main__":
    main()
