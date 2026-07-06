#!/usr/bin/env python3
"""Offline field dashboard for the Mu detector.

This app intentionally uses only the Python standard library. It talks to Mu
through ST-Link/OpenOCD, polls the live SRAM status block, and can retrieve the
external flash log through Mu's SRAM flash-read mailbox.
"""

from __future__ import annotations

import csv
import datetime as dt
import json
import os
import queue
import re
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Iterable

try:
    import tkinter as tk
    from tkinter import filedialog, ttk
except ModuleNotFoundError as exc:
    system_python = "/usr/bin/python3"
    if (
        exc.name == "_tkinter"
        and os.path.exists(system_python)
        and os.path.realpath(sys.executable) != os.path.realpath(system_python)
    ):
        os.execv(system_python, [system_python, *sys.argv])
    raise


REPO_ROOT = Path(__file__).resolve().parents[1]
FIRMWARE_ROOT = REPO_ROOT / "firmware"
DEMO_ROOT = REPO_ROOT / "demo"
RUNS_ROOT = DEMO_ROOT / "runs"

OPENOCD = Path.home() / ".platformio/packages/tool-openocd/bin/openocd"
NM = Path.home() / ".platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm"
PIO = shutil.which("pio") or "pio"

FLIGHT_ENV = "mu_f042_flight_logger"
ERASE_ENV = "mu_f042_flight_logger_erase"

STATUS_MAGIC = 0x4D55464C  # MUFL
MAILBOX_MAGIC = 0x4D554442  # MUDB
RECORD_MAGIC = 0x4D555245  # MURE
FLASH_LOG_MAGIC = 0x48495052
FLASH_LOG_COMMITTED = 0x00000000
FLASH_LOG_UNCOMMITTED = 0xFFFFFFFF
FNV1A_OFFSET = 2166136261
FNV1A_PRIME = 16777619

PAYLOAD_BOOT = 100
PAYLOAD_PRESSURE = 101
PAYLOAD_EVENT = 102

MAILBOX_WORDS_BEFORE_BUFFER = 12
MAILBOX_BUFFER_OFFSET = MAILBOX_WORDS_BEFORE_BUFFER * 4
MAILBOX_COMMAND_READ_FLASH = 1
MAILBOX_STATE_IDLE = 0
MAILBOX_STATE_BUSY = 1
MAILBOX_STATE_DONE = 2
MAILBOX_STATE_ERROR = 3
MAILBOX_RESULT_OK = 0

STATUS_FIELDS = [
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
    "firmware_flavor",
    "firmware_build_id",
    "log_start",
    "log_length",
    "log_used_bytes",
    "log_full",
    "flash_read_state",
    "flash_read_result",
    "flash_read_seq",
    "flash_read_bytes",
]

LEGACY_STATUS_FIELDS = STATUS_FIELDS[:38]

SIGNED_STATUS_FIELDS = {"pressure_pa", "temperature_centi_c", "altitude_mm"}

HEADER_STRUCT = struct.Struct("<IHHIIIHHIIII")
BOOT_STRUCT = struct.Struct("<15I")
PRESSURE_STRUCT = struct.Struct("<IIIiiiI")
EVENT_PREFIX_STRUCT = struct.Struct("<IIIIIIIIIIIiiiIIIII")


def signed32(value: int) -> int:
    return value - 0x100000000 if value & 0x80000000 else value


def pressure_height_m(pressure_pa: int | float) -> float:
    if pressure_pa <= 0:
        return 0.0
    return 44330.0 * (1.0 - (float(pressure_pa) / 101325.0) ** 0.190294957)


def align4(value: int) -> int:
    return (value + 3) & ~3


def fnv1a(data: bytes) -> int:
    value = FNV1A_OFFSET
    for byte in data:
        value ^= byte
        value = (value * FNV1A_PRIME) & 0xFFFFFFFF
    return value


def header_checksum(header_bytes: bytes) -> int:
    mutable = bytearray(header_bytes)
    struct.pack_into("<I", mutable, 32, 0)
    struct.pack_into("<I", mutable, 36, FLASH_LOG_UNCOMMITTED)
    return fnv1a(bytes(mutable))


def parse_words(text: str) -> list[int]:
    words: list[int] = []
    for line in text.splitlines():
        if not line.strip().startswith("0x"):
            continue
        hex_words = re.findall(r"\b[0-9a-fA-F]{8}\b", line)
        if hex_words and line.strip().startswith("0x" + hex_words[0]):
            hex_words = hex_words[1:]
        words.extend(int(word, 16) for word in hex_words)
    return words


def words_to_bytes(words: Iterable[int], wanted: int) -> bytes:
    data = bytearray()
    for word in words:
        data.extend(struct.pack("<I", word & 0xFFFFFFFF))
    return bytes(data[:wanted])


def run_process(
    cmd: list[str],
    cwd: Path,
    log: Callable[[str], None],
    timeout: float | None = None,
) -> int:
    log("$ " + " ".join(cmd))
    proc = subprocess.Popen(
        cmd,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
    )
    assert proc.stdout is not None
    start = time.time()
    for line in proc.stdout:
        log(line.rstrip())
        if timeout is not None and time.time() - start > timeout:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
            log("process timed out")
            return 124
    return proc.wait()


def ensure_elf(env: str, log: Callable[[str], None]) -> Path:
    elf = FIRMWARE_ROOT / ".pio" / "build" / env / "firmware.elf"
    if elf.exists():
        return elf
    log(f"{env} ELF missing; building it now")
    rc = run_process([PIO, "run", "-e", env], FIRMWARE_ROOT, log)
    if rc != 0:
        raise RuntimeError(f"pio build failed for {env} rc={rc}")
    if not elf.exists():
        raise RuntimeError(f"ELF still missing after build: {elf}")
    return elf


def symbol_addresses(env: str, names: set[str], log: Callable[[str], None]) -> dict[str, int]:
    elf = ensure_elf(env, log)
    proc = subprocess.run(
        [str(NM), "-S", str(elf)],
        cwd=FIRMWARE_ROOT,
        text=True,
        capture_output=True,
        check=True,
    )
    symbols: dict[str, int] = {}
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[3] in names:
            symbols[parts[3]] = int(parts[0], 16)
    missing = names - set(symbols)
    if missing:
        raise RuntimeError(f"missing symbols in {env}: {sorted(missing)}")
    return symbols


class OpenOCDSession:
    def __init__(self, log: Callable[[str], None]) -> None:
        self.log = log
        self.proc: subprocess.Popen[str] | None = None
        self.sock: socket.socket | None = None
        self.output: list[str] = []
        self.lock = threading.Lock()
        self.telnet_port, self.gdb_port, self.tcl_port = self._free_ports(3)

    @staticmethod
    def _free_ports(count: int) -> list[int]:
        sockets: list[socket.socket] = []
        try:
            ports: list[int] = []
            for _ in range(count):
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.bind(("127.0.0.1", 0))
                sockets.append(s)
                ports.append(int(s.getsockname()[1]))
            return ports
        finally:
            for s in sockets:
                s.close()

    def start(self) -> None:
        if self.sock is not None:
            return
        if not OPENOCD.exists():
            raise RuntimeError(f"OpenOCD not found: {OPENOCD}")

        cmd = [
            str(OPENOCD),
            "-f",
            "interface/stlink.cfg",
            "-f",
            "target/stm32f0x.cfg",
            "-c",
            f"telnet_port {self.telnet_port}",
            "-c",
            f"gdb_port {self.gdb_port}",
            "-c",
            f"tcl_port {self.tcl_port}",
            "-c",
            "init",
        ]
        self.log("$ " + " ".join(cmd))
        self.proc = subprocess.Popen(
            cmd,
            cwd=FIRMWARE_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
        )
        threading.Thread(target=self._collect_output, daemon=True).start()

        deadline = time.time() + 8.0
        last_error = ""
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError("OpenOCD exited: " + "\n".join(self.output[-12:]))
            try:
                self.sock = socket.create_connection(
                    ("127.0.0.1", self.tcl_port), timeout=0.5
                )
                self.sock.settimeout(4.0)
                self._raw_command("rbp all", timeout=3.0, tolerate_error=True)
                self._raw_command("resume", timeout=3.0, tolerate_error=True)
                self.log("OpenOCD connected via TCL")
                return
            except OSError as exc:
                last_error = str(exc)
                time.sleep(0.2)
        self.stop()
        raise RuntimeError(f"could not connect to OpenOCD TCL: {last_error}")

    def _collect_output(self) -> None:
        if self.proc is None or self.proc.stdout is None:
            return
        for line in self.proc.stdout:
            line = line.rstrip()
            self.output.append(line)
            if len(self.output) > 200:
                self.output = self.output[-100:]

    def _read_tcl_response(self, timeout: float = 4.0) -> str:
        assert self.sock is not None
        old_timeout = self.sock.gettimeout()
        self.sock.settimeout(timeout)
        chunks: list[bytes] = []
        try:
            while True:
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise RuntimeError("OpenOCD socket closed")
                chunks.append(chunk)
                data = b"".join(chunks)
                if b"\x1a" in data:
                    data = data.split(b"\x1a", 1)[0]
                    return data.decode("utf-8", errors="ignore")
        finally:
            self.sock.settimeout(old_timeout)

    def _raw_command(self, command: str, timeout: float = 8.0, tolerate_error: bool = False) -> str:
        assert self.sock is not None
        self.sock.settimeout(timeout)
        self.sock.sendall(command.encode("utf-8") + b"\x1a")
        text = self._read_tcl_response(timeout=timeout)
        if not tolerate_error and "Error:" in text:
            raise RuntimeError(text.strip())
        return text

    def command(self, command: str, timeout: float = 8.0, tolerate_error: bool = False) -> str:
        with self.lock:
            self.start()
            return self._raw_command(command, timeout=timeout, tolerate_error=tolerate_error)

    def read_words(self, address: int, count: int, timeout: float = 8.0) -> list[int]:
        max_words_per_mdw = 8
        if count > max_words_per_mdw:
            words: list[int] = []
            current = address
            remaining = count
            while remaining > 0:
                chunk = min(max_words_per_mdw, remaining)
                words.extend(self.read_words(current, chunk, timeout=timeout))
                current += chunk * 4
                remaining -= chunk
            return words

        last_text = ""
        last_count = 0
        for _attempt in range(2):
            text = self.command(f"mdw 0x{address:08x} {count}", timeout=timeout)
            words = parse_words(text)
            if len(words) >= count:
                return words[:count]
            last_text = text
            last_count = len(words)
            time.sleep(0.05)
        raise RuntimeError(f"mdw returned {last_count} words, wanted {count}: {last_text}")

    def write_word(self, address: int, value: int, timeout: float = 8.0) -> None:
        self.command(f"mww 0x{address:08x} 0x{value & 0xFFFFFFFF:08x}", timeout=timeout)

    def stop(self) -> None:
        sock = self.sock
        self.sock = None
        if sock is not None:
            try:
                sock.sendall(b"shutdown\n")
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        if self.proc is not None:
            if self.proc.poll() is None:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
            self.proc = None


class MuLink:
    def __init__(self, log: Callable[[str], None]) -> None:
        self.log = log
        self.session: OpenOCDSession | None = None
        self.status_addr = 0
        self.mailbox_addr = 0
        self.seq = 1
        self.status_mode = "auto"

    def connect(self) -> None:
        if self.session is None:
            self.session = OpenOCDSession(self.log)
        self.session.start()
        if self.status_addr == 0 or self.mailbox_addr == 0:
            symbols = symbol_addresses(
                FLIGHT_ENV,
                {"g_mu_flight", "g_mu_flash_mailbox"},
                self.log,
            )
            self.status_addr = symbols["g_mu_flight"]
            self.mailbox_addr = symbols["g_mu_flash_mailbox"]
            self.log(
                f"symbols: g_mu_flight=0x{self.status_addr:08x} "
                f"g_mu_flash_mailbox=0x{self.mailbox_addr:08x}"
            )

    def close(self) -> None:
        if self.session is not None:
            self.session.stop()
            self.session = None

    def read_status(self) -> dict[str, int]:
        self.connect()
        assert self.session is not None

        def read_v2_status() -> dict[str, int]:
            words = self.session.read_words(self.status_addr, len(STATUS_FIELDS), timeout=8.0)
            data = dict(zip(STATUS_FIELDS, words))
            if data.get("magic") != STATUS_MAGIC:
                raise RuntimeError(f"bad status magic at v2 address: 0x{data.get('magic', 0):08x}")
            return data

        def read_legacy_status() -> dict[str, int]:
            words = self.session.read_words(self.mailbox_addr, len(LEGACY_STATUS_FIELDS), timeout=8.0)
            data = dict(zip(LEGACY_STATUS_FIELDS, words))
            if data.get("magic") != STATUS_MAGIC:
                raise RuntimeError(f"bad legacy status magic: 0x{data.get('magic', 0):08x}")
            data.update(
                {
                    "firmware_flavor": 0,
                    "firmware_build_id": 0,
                    "log_start": 0,
                    "log_length": 0,
                    "log_used_bytes": 0,
                    "log_full": 0,
                    "flash_read_state": 0,
                    "flash_read_result": 0,
                    "flash_read_seq": 0,
                    "flash_read_bytes": 0,
                }
            )
            return data

        try:
            data = read_v2_status()
            if self.status_mode == "legacy":
                self.log("v2 status detected; dashboard readout firmware is active")
            self.status_mode = "v2"
        except Exception as exc:
            data = read_legacy_status()
            if self.status_mode != "legacy":
                self.log(f"legacy v1 status detected; flash v2 for dashboard readout ({exc})")
            self.status_mode = "legacy"
        for name in SIGNED_STATUS_FIELDS:
            data[name] = signed32(data[name])
        return data

    def read_mailbox_header(self) -> dict[str, int]:
        self.connect()
        assert self.session is not None
        names = [
            "magic",
            "version",
            "command",
            "state",
            "request_seq",
            "response_seq",
            "address",
            "length",
            "bytes_read",
            "result",
            "log_start",
            "log_length",
        ]
        words = self.session.read_words(self.mailbox_addr, len(names), timeout=8.0)
        return dict(zip(names, words))

    def read_flash_chunk(self, address: int, length: int) -> bytes:
        if length <= 0 or length > 512:
            raise ValueError("mailbox length must be 1..512")
        self.connect()
        assert self.session is not None
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        if self.seq == 0:
            self.seq = 1

        base = self.mailbox_addr
        self.session.write_word(base + 12, MAILBOX_STATE_IDLE)
        self.session.write_word(base + 32, 0)
        self.session.write_word(base + 36, 0)
        self.session.write_word(base + 24, address)
        self.session.write_word(base + 28, length)
        self.session.write_word(base + 16, seq)
        self.session.write_word(base + 8, MAILBOX_COMMAND_READ_FLASH)

        deadline = time.time() + 5.0
        header: dict[str, int] | None = None
        while time.time() < deadline:
            header = self.read_mailbox_header()
            if header["response_seq"] == seq and header["state"] in (
                MAILBOX_STATE_DONE,
                MAILBOX_STATE_ERROR,
            ):
                break
            time.sleep(0.02)
        else:
            raise RuntimeError(f"flash mailbox timeout seq={seq}")

        assert header is not None
        if header["state"] != MAILBOX_STATE_DONE or header["result"] != MAILBOX_RESULT_OK:
            raise RuntimeError(
                f"flash mailbox failed seq={seq} state={header['state']} "
                f"result={header['result']}"
            )
        bytes_read = header["bytes_read"]
        word_count = (bytes_read + 3) // 4
        words = self.session.read_words(
            base + MAILBOX_BUFFER_OFFSET,
            word_count,
            timeout=8.0,
        )
        return words_to_bytes(words, bytes_read)


def firmware_label(status: dict[str, int] | None) -> tuple[str, str]:
    if status is None:
        return "disconnected", "bad"
    if status.get("magic") != STATUS_MAGIC:
        return "incompatible", "bad"
    version = status.get("version", 0)
    flavor = status.get("firmware_flavor", 0)
    build = status.get("firmware_build_id", 0)
    if version < 2:
        return f"flight logger legacy v{version}", "warn"
    if flavor == 1:
        return f"flight logger v{version} build {build}", "good"
    if flavor == 2:
        return f"erase image v{version} build {build}", "warn"
    return f"unknown Mu firmware v{version}", "warn"


def status_is_booting(status: dict[str, int]) -> bool:
    if status.get("magic") != STATUS_MAGIC:
        return False
    if status.get("uptime_ms", 0) != 0 and status.get("sample_count", 0) != 0:
        return False
    return status.get("logger_ok", 0) == 0 or status.get("sample_count", 0) == 0


class LinePlot(tk.Canvas):
    def __init__(self, parent: tk.Widget, height: int = 145) -> None:
        super().__init__(
            parent,
            height=height,
            bg="#111318",
            highlightthickness=1,
            highlightbackground="#2d3138",
        )
        self.series: list[tuple[str, list[float], list[float], str]] = []
        self.title = ""
        self.ylabel = ""
        self.bind("<Configure>", lambda _event: self.draw())

    def set_data(
        self,
        title: str,
        ylabel: str,
        series: list[tuple[str, list[float], list[float], str]],
    ) -> None:
        self.title = title
        self.ylabel = ylabel
        self.series = series
        self.draw()

    def draw(self) -> None:
        self.delete("all")
        width = max(self.winfo_width(), 320)
        height = max(self.winfo_height(), 120)
        left, right, top, bottom = 54, 16, 24, 28
        plot_w = width - left - right
        plot_h = height - top - bottom
        self.create_text(
            width // 2,
            12,
            text=self.title,
            fill="#e7ecf3",
            font=("Helvetica", 12, "bold"),
        )
        self.create_rectangle(left, top, left + plot_w, top + plot_h, outline="#39404b")
        all_x = [x for _label, xs, _ys, _color in self.series for x in xs]
        all_y = [y for _label, _xs, ys, _color in self.series for y in ys]
        if not all_x or not all_y:
            self.create_text(
                width // 2,
                height // 2,
                text="waiting for data",
                fill="#79808d",
                font=("Helvetica", 11),
            )
            return
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        if max_x <= min_x:
            max_x = min_x + 1.0
        if max_y <= min_y:
            pad = max(1.0, abs(max_y) * 0.1)
            min_y -= pad
            max_y += pad
        else:
            pad = (max_y - min_y) * 0.12
            min_y -= pad
            max_y += pad

        def sx(value: float) -> float:
            return left + (value - min_x) * plot_w / (max_x - min_x)

        def sy(value: float) -> float:
            return top + (max_y - value) * plot_h / (max_y - min_y)

        for frac in (0.25, 0.5, 0.75):
            y = top + plot_h * frac
            self.create_line(left, y, left + plot_w, y, fill="#222832")
        self.create_text(6, top + plot_h / 2, text=self.ylabel, angle=90, fill="#8b93a3")
        self.create_text(left - 6, top + 8, text=f"{max_y:.1f}", anchor="e", fill="#8b93a3")
        self.create_text(
            left - 6,
            top + plot_h - 8,
            text=f"{min_y:.1f}",
            anchor="e",
            fill="#8b93a3",
        )
        self.create_text(left, top + plot_h + 16, text=f"{min_x:.1f}", fill="#8b93a3")
        self.create_text(
            left + plot_w,
            top + plot_h + 16,
            text=f"{max_x:.1f}",
            fill="#8b93a3",
        )

        legend_x = left + 8
        for label, xs, ys, color in self.series:
            if len(xs) < 2 or len(ys) < 2:
                continue
            points: list[float] = []
            for x, y in zip(xs, ys):
                points.extend([sx(x), sy(y)])
            self.create_line(*points, fill=color, width=2, smooth=True)
            self.create_text(legend_x, top + 12, text=label, anchor="w", fill=color)
            legend_x += max(70, len(label) * 8)


class DualAxisPlot(tk.Canvas):
    def __init__(self, parent: tk.Widget, height: int = 180) -> None:
        super().__init__(
            parent,
            height=height,
            bg="#111318",
            highlightthickness=1,
            highlightbackground="#2d3138",
        )
        self.title = ""
        self.left_label = ""
        self.right_label = ""
        self.left_series: list[tuple[str, list[float], list[float], str]] = []
        self.right_series: list[tuple[str, list[float], list[float], str]] = []
        self.bind("<Configure>", lambda _event: self.draw())

    def set_data(
        self,
        title: str,
        left_label: str,
        right_label: str,
        left_series: list[tuple[str, list[float], list[float], str]],
        right_series: list[tuple[str, list[float], list[float], str]],
    ) -> None:
        self.title = title
        self.left_label = left_label
        self.right_label = right_label
        self.left_series = left_series
        self.right_series = right_series
        self.draw()

    def draw(self) -> None:
        self.delete("all")
        width = max(self.winfo_width(), 360)
        height = max(self.winfo_height(), 140)
        left, right, top, bottom = 60, 62, 24, 30
        plot_w = width - left - right
        plot_h = height - top - bottom
        self.create_text(
            width // 2,
            12,
            text=self.title,
            fill="#e7ecf3",
            font=("Helvetica", 12, "bold"),
        )
        self.create_rectangle(left, top, left + plot_w, top + plot_h, outline="#39404b")

        all_x = [
            x
            for series in (self.left_series, self.right_series)
            for _label, xs, _ys, _color in series
            for x in xs
        ]
        left_y = [y for _label, _xs, ys, _color in self.left_series for y in ys]
        right_y = [y for _label, _xs, ys, _color in self.right_series for y in ys]
        if not all_x or (not left_y and not right_y):
            self.create_text(width // 2, height // 2, text="waiting for data", fill="#79808d")
            return

        min_x, max_x = min(all_x), max(all_x)
        if max_x <= min_x:
            max_x = min_x + 1.0

        def bounds(values: list[float]) -> tuple[float, float]:
            if not values:
                return 0.0, 1.0
            lo, hi = min(values), max(values)
            if hi <= lo:
                pad = max(1.0, abs(hi) * 0.1)
                return lo - pad, hi + pad
            pad = (hi - lo) * 0.12
            return lo - pad, hi + pad

        left_min, left_max = bounds(left_y)
        right_min, right_max = bounds(right_y)

        def sx(value: float) -> float:
            return left + (value - min_x) * plot_w / (max_x - min_x)

        def sy_left(value: float) -> float:
            return top + (left_max - value) * plot_h / (left_max - left_min)

        def sy_right(value: float) -> float:
            return top + (right_max - value) * plot_h / (right_max - right_min)

        for frac in (0.25, 0.5, 0.75):
            y = top + plot_h * frac
            self.create_line(left, y, left + plot_w, y, fill="#222832")
        self.create_text(8, top + plot_h / 2, text=self.left_label, angle=90, fill="#8b93a3")
        self.create_text(width - 12, top + plot_h / 2, text=self.right_label, angle=90, fill="#8b93a3")
        self.create_text(left - 6, top + 8, text=f"{left_max:.1f}", anchor="e", fill="#8b93a3")
        self.create_text(left - 6, top + plot_h - 8, text=f"{left_min:.1f}", anchor="e", fill="#8b93a3")
        self.create_text(left + plot_w + 6, top + 8, text=f"{right_max:.1f}", anchor="w", fill="#8b93a3")
        self.create_text(left + plot_w + 6, top + plot_h - 8, text=f"{right_min:.1f}", anchor="w", fill="#8b93a3")
        self.create_text(left, top + plot_h + 16, text=f"{min_x:.1f}", fill="#8b93a3")
        self.create_text(left + plot_w, top + plot_h + 16, text=f"{max_x:.1f}", fill="#8b93a3")

        legend_x = left + 8
        for label, xs, ys, color in self.left_series:
            if len(xs) >= 2 and len(ys) >= 2:
                points: list[float] = []
                for x, y in zip(xs, ys):
                    points.extend([sx(x), sy_left(y)])
                self.create_line(*points, fill=color, width=2, smooth=True)
            self.create_text(legend_x, top + 12, text=label, anchor="w", fill=color)
            legend_x += max(90, len(label) * 8)

        for label, xs, ys, color in self.right_series:
            if len(xs) >= 2 and len(ys) >= 2:
                points = []
                for x, y in zip(xs, ys):
                    points.extend([sx(x), sy_right(y)])
                self.create_line(*points, fill=color, width=2, smooth=True)
            self.create_text(legend_x, top + 12, text=label, anchor="w", fill=color)
            legend_x += max(90, len(label) * 8)


def parse_flash_dump(blob: bytes, log: Callable[[str], None]) -> dict[str, object]:
    offset = 0
    records: list[dict[str, object]] = []
    boots: list[dict[str, object]] = []
    pressures: list[dict[str, object]] = []
    events: list[dict[str, object]] = []
    warnings: list[str] = []

    while offset + HEADER_STRUCT.size <= len(blob):
        header_bytes = blob[offset : offset + HEADER_STRUCT.size]
        if all(byte == 0xFF for byte in header_bytes):
            break
        values = HEADER_STRUCT.unpack(header_bytes)
        header = {
            "address": offset,
            "magic": values[0],
            "version": values[1],
            "header_size": values[2],
            "run_id": values[3],
            "sequence": values[4],
            "timestamp_ms": values[5],
            "payload_type": values[6],
            "payload_version": values[7],
            "payload_length": values[8],
            "payload_checksum": values[9],
            "header_checksum": values[10],
            "commit_marker": values[11],
        }
        if header["magic"] != FLASH_LOG_MAGIC or header["header_size"] != HEADER_STRUCT.size:
            warnings.append(f"corrupt header at 0x{offset:06x}")
            break
        total_size = align4(int(header["header_size"]) + int(header["payload_length"]))
        if total_size <= 0 or offset + total_size > len(blob):
            warnings.append(f"truncated record at 0x{offset:06x}")
            break
        if header["commit_marker"] != FLASH_LOG_COMMITTED:
            warnings.append(f"incomplete record at 0x{offset:06x}")
            break
        expected_header = header_checksum(header_bytes)
        if expected_header != header["header_checksum"]:
            warnings.append(f"header checksum mismatch at 0x{offset:06x}")
        payload_start = offset + int(header["header_size"])
        payload_end = payload_start + int(header["payload_length"])
        payload = blob[payload_start:payload_end]
        if fnv1a(payload) != header["payload_checksum"]:
            warnings.append(f"payload checksum mismatch at 0x{offset:06x}")

        records.append(header)
        payload_type = int(header["payload_type"])
        try:
            if payload_type == PAYLOAD_BOOT and len(payload) >= BOOT_STRUCT.size:
                fields = BOOT_STRUCT.unpack(payload[: BOOT_STRUCT.size])
                boots.append(
                    {
                        **header,
                        "record_magic": fields[0],
                        "firmware_version": fields[1],
                        "system_hz": fields[2],
                        "boot_run_id": fields[3],
                        "flash_jedec": fields[4],
                        "log_start": fields[5],
                        "log_length": fields[6],
                        "sample_interval_us": fields[7],
                        "trigger_delta_raw": fields[8],
                        "trigger_consecutive_samples": fields[9],
                        "waveform_samples": fields[10],
                        "waveform_pre_samples": fields[11],
                        "pressure_period_ms": fields[12],
                        "erase_on_boot": fields[13],
                        "baro_ok": fields[14],
                    }
                )
            elif payload_type == PAYLOAD_PRESSURE and len(payload) >= PRESSURE_STRUCT.size:
                fields = PRESSURE_STRUCT.unpack(payload[: PRESSURE_STRUCT.size])
                pressures.append(
                    {
                        **header,
                        "record_magic": fields[0],
                        "pressure_seq": fields[1],
                        "timestamp_ms": fields[2],
                        "pressure_pa": fields[3],
                        "pressure_height_m": pressure_height_m(fields[3]),
                        "temperature_c": fields[4] / 100.0,
                        "altitude_m": fields[5] / 1000.0,
                        "ok": fields[6],
                    }
                )
            elif payload_type == PAYLOAD_EVENT and len(payload) >= EVENT_PREFIX_STRUCT.size:
                prefix = EVENT_PREFIX_STRUCT.unpack(payload[: EVENT_PREFIX_STRUCT.size])
                sample_count = max(0, (len(payload) - EVENT_PREFIX_STRUCT.size) // 2)
                samples = list(
                    struct.unpack(
                        f"<{sample_count}H",
                        payload[
                            EVENT_PREFIX_STRUCT.size : EVENT_PREFIX_STRUCT.size
                            + sample_count * 2
                        ],
                    )
                )
                events.append(
                    {
                        **header,
                        "record_magic": prefix[0],
                        "event_seq": prefix[1],
                        "timestamp_ms": prefix[2],
                        "sample_count": prefix[3],
                        "baseline_raw": prefix[4],
                        "trigger_raw": prefix[5],
                        "min_raw": prefix[6],
                        "max_raw": prefix[7],
                        "baseline_mv": prefix[8],
                        "min_mv": prefix[9],
                        "amplitude_mv": prefix[10],
                        "pressure_pa": prefix[11],
                        "pressure_height_m": pressure_height_m(prefix[11]),
                        "temperature_c": prefix[12] / 100.0,
                        "altitude_m": prefix[13] / 1000.0,
                        "pressure_age_ms": prefix[14],
                        "pressure_ok": prefix[15],
                        "waveform_count": prefix[16],
                        "waveform_pre_samples": prefix[17],
                        "sample_interval_us": prefix[18],
                        "samples": samples,
                    }
                )
        except struct.error as exc:
            warnings.append(f"payload parse failed at 0x{offset:06x}: {exc}")
        offset += total_size

    duration_s = 0.0
    if records:
        duration_s = max(0.0, (records[-1]["timestamp_ms"] - records[0]["timestamp_ms"]) / 1000.0)  # type: ignore[index,operator]
    run_ids = sorted({int(record["run_id"]) for record in records})
    summary = {
        "records": len(records),
        "boot_records": len(boots),
        "pressure_records": len(pressures),
        "event_records": len(events),
        "run_ids": run_ids,
        "latest_run_id": run_ids[-1] if run_ids else 0,
        "used_bytes": offset,
        "duration_s": duration_s,
        "event_rate_per_min": (len(events) / duration_s * 60.0) if duration_s > 0 else 0.0,
        "warnings": warnings,
    }
    for warning in warnings:
        log("readout warning: " + warning)
    return {
        "summary": summary,
        "records": records,
        "boots": boots,
        "pressures": pressures,
        "events": events,
    }


def raw_to_mv(raw: int) -> float:
    return raw * 3300.0 / 4095.0


def save_event_svg(path: Path, event: dict[str, object]) -> None:
    samples = event.get("samples", [])
    if not isinstance(samples, list) or not samples:
        return
    pre = int(event.get("waveform_pre_samples", 16))
    sample_interval_us = int(event.get("sample_interval_us", 10))
    times = [(i - pre + 1) * sample_interval_us for i in range(len(samples))]
    values = [raw_to_mv(int(raw)) for raw in samples]
    width, height = 900, 420
    left, right, top, bottom = 64, 24, 34, 50
    plot_w, plot_h = width - left - right, height - top - bottom
    min_t, max_t = min(times), max(times)
    min_v, max_v = min(values), max(values)
    pad_v = max(25.0, (max_v - min_v) * 0.14)
    min_v -= pad_v
    max_v += pad_v

    def sx(value: float) -> float:
        return left + (value - min_t) * plot_w / (max_t - min_t)

    def sy(value: float) -> float:
        return top + (max_v - value) * plot_h / (max_v - min_v)

    points = " ".join(f"{sx(t):.2f},{sy(v):.2f}" for t, v in zip(times, values))
    baseline_y = sy(float(event.get("baseline_mv", values[0])))
    zero_x = sx(0)
    title = f"Mu event {event.get('event_seq', '?')} amp {event.get('amplitude_mv', '?')} mV"
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width / 2}" y="23" text-anchor="middle" font-family="sans-serif" font-size="18">{title}</text>',
        f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="#222"/>',
        f'<line x1="{left}" y1="{baseline_y:.2f}" x2="{left + plot_w}" y2="{baseline_y:.2f}" stroke="#2ca02c" stroke-dasharray="7 5"/>',
        f'<line x1="{zero_x:.2f}" y1="{top}" x2="{zero_x:.2f}" y2="{top + plot_h}" stroke="#111" opacity="0.55"/>',
        f'<polyline points="{points}" fill="none" stroke="#1f77b4" stroke-width="1.5"/>',
        f'<text x="{width / 2}" y="{height - 14}" text-anchor="middle" font-family="sans-serif" font-size="12">time from trigger (us)</text>',
        f'<text x="18" y="{top + plot_h / 2}" transform="rotate(-90 18 {top + plot_h / 2})" text-anchor="middle" font-family="sans-serif" font-size="12">CATCH_OUT (mV)</text>',
        "</svg>",
    ]
    path.write_text("\n".join(parts), encoding="utf-8")


def save_run_bundle(parsed: dict[str, object], dump: bytes) -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = RUNS_ROOT / stamp
    wave_dir = out / "waveforms"
    wave_dir.mkdir(parents=True, exist_ok=True)
    (out / "flash_dump.bin").write_bytes(dump)
    summary = parsed["summary"]
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    def write_csv(path: Path, rows: list[dict[str, object]], skip: set[str] | None = None) -> None:
        skip = skip or set()
        keys: list[str] = []
        for row in rows:
            for key in row.keys():
                if key not in skip and key not in keys:
                    keys.append(key)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=keys)
            writer.writeheader()
            for row in rows:
                writer.writerow({key: row.get(key, "") for key in keys})

    write_csv(out / "records.csv", parsed["records"])  # type: ignore[arg-type]
    write_csv(out / "pressure.csv", parsed["pressures"])  # type: ignore[arg-type]
    write_csv(out / "events.csv", parsed["events"], skip={"samples"})  # type: ignore[arg-type]

    for event in parsed["events"]:  # type: ignore[union-attr]
        seq = int(event.get("event_seq", 0))
        sample_interval_us = int(event.get("sample_interval_us", 10))
        pre = int(event.get("waveform_pre_samples", 16))
        csv_path = wave_dir / f"event_{seq:04d}.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["sample", "time_us", "raw", "mv"])
            for i, raw in enumerate(event.get("samples", [])):
                writer.writerow([i, (i - pre + 1) * sample_interval_us, raw, f"{raw_to_mv(int(raw)):.3f}"])
        save_event_svg(wave_dir / f"event_{seq:04d}.svg", event)
    return out


def parse_csv_value(value: str) -> object:
    value = value.strip()
    if value == "":
        return ""
    if re.fullmatch(r"[-+]?\d+", value):
        try:
            return int(value)
        except ValueError:
            return value
    if re.fullmatch(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", value):
        try:
            return float(value)
        except ValueError:
            return value
    return value


def read_saved_csv(path: Path) -> list[dict[str, object]]:
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as handle:
        return [{key: parse_csv_value(value) for key, value in row.items()} for row in csv.DictReader(handle)]


def load_run_bundle(run_dir: Path, log: Callable[[str], None]) -> tuple[dict[str, object], bytes]:
    if not run_dir.is_dir():
        raise RuntimeError(f"{run_dir} is not a run directory")

    dump_path = run_dir / "flash_dump.bin"
    if dump_path.exists():
        dump = dump_path.read_bytes()
        return parse_flash_dump(dump, log), dump

    records = read_saved_csv(run_dir / "records.csv")
    pressures = read_saved_csv(run_dir / "pressure.csv")
    events = read_saved_csv(run_dir / "events.csv")
    if not records and not pressures and not events:
        raise RuntimeError(f"{run_dir} has no flash_dump.bin or saved CSV files")

    summary_path = run_dir / "summary.json"
    if summary_path.exists():
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
    else:
        duration_s = 0.0
        timestamps = [int(row.get("timestamp_ms", 0)) for row in records if row.get("timestamp_ms") != ""]
        if timestamps:
            duration_s = max(0.0, (max(timestamps) - min(timestamps)) / 1000.0)
        summary = {
            "latest_run_id": 0,
            "records": len(records),
            "event_records": len(events),
            "pressure_records": len(pressures),
            "duration_s": duration_s,
            "event_rate_per_min": (len(events) / duration_s * 60.0) if duration_s > 0 else 0.0,
            "used_bytes": 0,
            "warnings": [],
        }

    warnings = summary.get("warnings", [])
    if not isinstance(warnings, list):
        warnings = [str(warnings)]
    warnings.append("loaded from CSV fallback; event waveform samples unavailable")
    summary["warnings"] = warnings
    return {
        "summary": summary,
        "records": records,
        "boots": [],
        "pressures": pressures,
        "events": events,
    }, b""


class DashboardApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Mu Offline Field Dashboard")
        self.root.geometry("1320x860")
        self.queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.link = MuLink(self.thread_log)
        self.stop_event = threading.Event()
        self.busy = threading.Event()
        self.live_paused = threading.Event()
        self.live_generation = 0
        self.live_history: list[tuple[float, dict[str, int]]] = []
        self.current_status: dict[str, int] | None = None
        self.current_readout: dict[str, object] | None = None
        self.action_buttons: list[ttk.Button] = []
        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        threading.Thread(target=self.live_loop, daemon=True).start()
        self.root.after(100, self.drain_queue)

    def _build_ui(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Good.TLabel", foreground="#0b7f39")
        style.configure("Warn.TLabel", foreground="#a66b00")
        style.configure("Bad.TLabel", foreground="#b00020")

        toolbar = ttk.Frame(self.root, padding=(8, 6))
        toolbar.pack(side=tk.TOP, fill=tk.X)
        self.connection_var = tk.StringVar(value="disconnected")
        self.firmware_var = tk.StringVar(value="unknown")
        self.pause_button_var = tk.StringVar(value="Pause Live Feed")
        self.connection_label = ttk.Label(toolbar, textvariable=self.connection_var, style="Bad.TLabel")
        self.connection_label.pack(side=tk.LEFT, padx=(0, 12))
        self.firmware_label = ttk.Label(toolbar, textvariable=self.firmware_var, style="Warn.TLabel")
        self.firmware_label.pack(side=tk.LEFT, padx=(0, 12))
        reconnect_btn = ttk.Button(toolbar, text="Reconnect", command=self.reconnect)
        reconnect_btn.pack(side=tk.LEFT, padx=3)
        pause_btn = ttk.Button(toolbar, textvariable=self.pause_button_var, command=self.toggle_live_pause)
        pause_btn.pack(side=tk.LEFT, padx=3)
        clear_btn = ttk.Button(toolbar, text="Clear", command=self.clear_live_data)
        clear_btn.pack(side=tk.LEFT, padx=3)
        flash_btn = ttk.Button(toolbar, text="Flash Flight Logger", command=self.flash_flight)
        flash_btn.pack(side=tk.LEFT, padx=3)
        wipe_btn = ttk.Button(toolbar, text="Wipe Flash + Restore Flight", command=self.wipe_restore)
        wipe_btn.pack(side=tk.LEFT, padx=3)
        retrieve_btn = ttk.Button(toolbar, text="Retrieve Data From Flash", command=self.retrieve_flash)
        retrieve_btn.pack(side=tk.LEFT, padx=3)
        load_btn = ttk.Button(toolbar, text="Load Saved Run", command=self.load_saved_run)
        load_btn.pack(side=tk.LEFT, padx=3)
        self.action_buttons = [reconnect_btn, flash_btn, wipe_btn, retrieve_btn, load_btn]

        paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        live = ttk.Frame(paned, padding=8)
        readout = ttk.Frame(paned, padding=8)
        paned.add(live, weight=1)
        paned.add(readout, weight=1)

        metrics = ttk.LabelFrame(live, text="Live Health", padding=8)
        metrics.pack(side=tk.TOP, fill=tk.X)
        self.metric_vars: dict[str, tk.StringVar] = {}
        metric_names = [
            "uptime",
            "baseline",
            "latest",
            "rate",
            "events",
            "records",
            "pressure",
            "temp",
            "altitude",
            "baro height",
            "queue",
            "errors",
            "storage",
        ]
        for idx, name in enumerate(metric_names):
            ttk.Label(metrics, text=name).grid(row=idx // 4 * 2, column=idx % 4, sticky="w", padx=6)
            var = tk.StringVar(value="-")
            self.metric_vars[name] = var
            ttk.Label(metrics, textvariable=var, font=("Helvetica", 12, "bold")).grid(
                row=idx // 4 * 2 + 1,
                column=idx % 4,
                sticky="w",
                padx=6,
                pady=(0, 5),
            )

        self.rate_plot = LinePlot(live)
        self.rate_plot.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(8, 0))
        self.voltage_plot = LinePlot(live)
        self.voltage_plot.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(8, 0))
        self.pressure_plot = LinePlot(live)
        self.pressure_plot.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(8, 0))

        summary_frame = ttk.LabelFrame(readout, text="Flash Readout", padding=8)
        summary_frame.pack(side=tk.TOP, fill=tk.X)
        self.readout_summary_var = tk.StringVar(value="No flash readout loaded.")
        ttk.Label(summary_frame, textvariable=self.readout_summary_var, justify=tk.LEFT).pack(fill=tk.X)

        plots = ttk.PanedWindow(readout, orient=tk.VERTICAL)
        plots.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(8, 0))
        event_frame = ttk.Frame(plots)
        experiment_frame = ttk.Frame(plots)
        plots.add(event_frame, weight=1)
        plots.add(experiment_frame, weight=1)

        list_frame = ttk.Frame(event_frame)
        list_frame.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(list_frame, text="Events").pack(anchor="w")
        self.event_list = tk.Listbox(list_frame, width=34, height=12)
        self.event_list.pack(side=tk.LEFT, fill=tk.Y)
        self.event_list.bind("<<ListboxSelect>>", self.on_event_selected)
        scroll = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.event_list.yview)
        scroll.pack(side=tk.LEFT, fill=tk.Y)
        self.event_list.configure(yscrollcommand=scroll.set)
        self.waveform_plot = LinePlot(event_frame, height=240)
        self.waveform_plot.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))

        self.experiment_time_plot = DualAxisPlot(experiment_frame, height=145)
        self.experiment_time_plot.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.experiment_correlation_plot = LinePlot(experiment_frame, height=145)
        self.experiment_correlation_plot.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(8, 0))

        console_frame = ttk.LabelFrame(self.root, text="Activity Console", padding=6)
        console_frame.pack(side=tk.BOTTOM, fill=tk.BOTH)
        self.console = tk.Text(console_frame, height=10, bg="#090b0f", fg="#d7dde8", insertbackground="white")
        self.console.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        console_scroll = ttk.Scrollbar(console_frame, orient=tk.VERTICAL, command=self.console.yview)
        console_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.console.configure(yscrollcommand=console_scroll.set)

    def thread_log(self, message: str) -> None:
        self.queue.put(("log", message))

    def log(self, message: str) -> None:
        stamp = dt.datetime.now().strftime("%H:%M:%S")
        self.console.insert(tk.END, f"[{stamp}] {message}\n")
        self.console.see(tk.END)

    def drain_queue(self) -> None:
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "log":
                    self.log(str(payload))
                elif kind == "status":
                    generation, status = payload  # type: ignore[misc]
                    if generation == self.live_generation and not self.busy.is_set():
                        self.apply_status(status)  # type: ignore[arg-type]
                elif kind == "disconnected":
                    generation, reason = payload  # type: ignore[misc]
                    if generation == self.live_generation and not self.busy.is_set():
                        self.apply_disconnected(str(reason))
                elif kind == "readout":
                    parsed, dump, out = payload  # type: ignore[misc]
                    self.apply_readout(parsed, dump, out)
                elif kind == "message":
                    self.log(str(payload))
                elif kind == "error":
                    self.log("ERROR: " + str(payload))
                    self.connection_var.set("error")
                    self.connection_label.configure(style="Bad.TLabel")
                elif kind == "operation_done":
                    self.finish_operation(str(payload))
        except queue.Empty:
            pass
        self.root.after(100, self.drain_queue)

    def set_action_buttons_enabled(self, enabled: bool) -> None:
        state = tk.NORMAL if enabled else tk.DISABLED
        for button in self.action_buttons:
            button.configure(state=state)

    def begin_exclusive_operation(self, label: str) -> bool:
        if self.busy.is_set():
            self.log("busy; wait for current operation to finish")
            return False
        self.busy.set()
        self.live_paused.set()
        self.live_generation += 1
        self.link.close()
        self.pause_button_var.set("Resume Live Feed")
        self.connection_var.set(label)
        self.connection_label.configure(style="Warn.TLabel")
        self.set_action_buttons_enabled(False)
        self.log(f"{label}; live feed paused and ST-Link released")
        return True

    def finish_operation(self, message: str) -> None:
        self.busy.clear()
        self.set_action_buttons_enabled(True)
        if message:
            self.log(message)

    def live_loop(self) -> None:
        while not self.stop_event.is_set():
            if self.live_paused.is_set():
                time.sleep(0.25)
                continue
            if self.busy.is_set():
                time.sleep(0.5)
                continue
            generation = self.live_generation
            try:
                status = self.link.read_status()
                self.queue.put(("status", (generation, status)))
                time.sleep(2.0)
            except Exception as exc:
                self.link.close()
                self.queue.put(("disconnected", (generation, exc)))
                time.sleep(3.0)

    def toggle_live_pause(self) -> None:
        if self.live_paused.is_set():
            self.live_paused.clear()
            self.live_generation += 1
            self.link.status_mode = "auto"
            self.pause_button_var.set("Pause Live Feed")
            self.log("live feed resumed")
        else:
            self.live_paused.set()
            self.live_generation += 1
            self.link.close()
            self.pause_button_var.set("Resume Live Feed")
            self.connection_var.set("live feed paused")
            self.connection_label.configure(style="Warn.TLabel")
            self.log("live feed paused; ST-Link released for flashing/debugging")

    def clear_live_data(self) -> None:
        self.live_history.clear()
        self.metric_vars["rate"].set("0.0 / min")
        self.redraw_live_plots()
        self.log("recent live data cleared")

    def apply_disconnected(self, reason: str) -> None:
        self.connection_var.set("disconnected")
        self.connection_label.configure(style="Bad.TLabel")
        self.firmware_var.set("no Mu status")
        self.firmware_label.configure(style="Bad.TLabel")
        if reason:
            self.log("live read failed: " + reason)

    def apply_status(self, status: dict[str, int]) -> None:
        self.current_status = status
        label, severity = firmware_label(status)
        self.firmware_var.set(label)
        self.firmware_label.configure(
            style={"good": "Good.TLabel", "warn": "Warn.TLabel", "bad": "Bad.TLabel"}[severity]
        )

        if status_is_booting(status):
            self.connection_var.set("booting / log scan")
            self.connection_label.configure(style="Warn.TLabel")
            self.live_history.clear()
            for var in self.metric_vars.values():
                var.set("...")
            length = status.get("log_length", 0)
            if length:
                self.metric_vars["storage"].set(f"scanning / {length} B")
            self.redraw_live_plots()
            return

        self.connection_var.set("connected")
        self.connection_label.configure(style="Good.TLabel")

        now = time.monotonic()
        if self.live_history and status["uptime_ms"] < self.live_history[-1][1]["uptime_ms"]:
            self.live_history.clear()
        self.live_history.append((now, status))
        self.live_history = self.live_history[-240:]

        rate = self.compute_rate()
        used = status.get("log_used_bytes", 0)
        length = status.get("log_length", 0)
        pct = used / length * 100.0 if length else 0.0
        errors = (
            status.get("event_queue_drops", 0)
            + status.get("dma_error_count", 0)
            + status.get("adc_overrun_count", 0)
            + status.get("event_log_failures", 0)
            + status.get("pressure_log_failures", 0)
        )
        self.metric_vars["uptime"].set(f"{status['uptime_ms'] / 1000.0:.1f} s")
        self.metric_vars["baseline"].set(f"{status['baseline_mv']} mV")
        self.metric_vars["latest"].set(f"{status['latest_mv']} mV")
        self.metric_vars["rate"].set(f"{rate:.1f} / min")
        self.metric_vars["events"].set(str(status["event_count"]))
        self.metric_vars["records"].set(str(status["records_written"]))
        self.metric_vars["pressure"].set(f"{status['pressure_pa']} Pa")
        self.metric_vars["temp"].set(f"{status['temperature_centi_c'] / 100.0:.2f} C")
        self.metric_vars["altitude"].set(f"{status['altitude_mm'] / 1000.0:.2f} m")
        self.metric_vars["baro height"].set(f"{pressure_height_m(status['pressure_pa']):.1f} m")
        self.metric_vars["queue"].set(f"{status['event_queue_depth']}/{status['event_queue_max_depth']}")
        self.metric_vars["errors"].set(str(errors))
        self.metric_vars["storage"].set(f"{used}/{length} B ({pct:.1f}%)")
        self.redraw_live_plots()

    def compute_rate(self) -> float:
        if len(self.live_history) < 2:
            return 0.0
        cutoff = self.live_history[-1][0] - 60.0
        window = [item for item in self.live_history if item[0] >= cutoff]
        if len(window) < 2:
            window = self.live_history
        dt_s = window[-1][0] - window[0][0]
        if dt_s <= 0:
            return 0.0
        de = window[-1][1]["event_count"] - window[0][1]["event_count"]
        return max(0.0, de / dt_s * 60.0)

    def redraw_live_plots(self) -> None:
        if not self.live_history:
            self.rate_plot.set_data("Event Rate", "events/min", [])
            self.voltage_plot.set_data("CATCH_OUT", "mV", [])
            self.pressure_plot.set_data("Pressure / Altitude", "kPa / m", [])
            return
        t0 = self.live_history[0][0]
        xs = [(t - t0) / 60.0 for t, _status in self.live_history]
        rates: list[float] = []
        for i, (t, status) in enumerate(self.live_history):
            if i == 0:
                rates.append(0.0)
                continue
            prev_t, prev_status = self.live_history[i - 1]
            dt_s = max(0.001, t - prev_t)
            rates.append(max(0.0, (status["event_count"] - prev_status["event_count"]) / dt_s * 60.0))
        baselines = [status["baseline_mv"] for _t, status in self.live_history]
        latest = [status["latest_mv"] for _t, status in self.live_history]
        pressures = [status["pressure_pa"] / 1000.0 for _t, status in self.live_history]
        altitude = [status["altitude_mm"] / 1000.0 for _t, status in self.live_history]
        pressure_height = [pressure_height_m(status["pressure_pa"]) for _t, status in self.live_history]
        self.rate_plot.set_data("Event Rate", "events/min", [("rate", xs, rates, "#65d46e")])
        self.voltage_plot.set_data(
            "CATCH_OUT",
            "mV",
            [("baseline", xs, baselines, "#56a8ff"), ("latest", xs, latest, "#f0c674")],
        )
        self.pressure_plot.set_data(
            "Pressure / Altitude",
            "kPa / m",
            [
                ("pressure kPa", xs, pressures, "#b892ff"),
                ("fw altitude m", xs, altitude, "#ff7f6e"),
                ("baro height m", xs, pressure_height, "#65d46e"),
            ],
        )

    def reconnect(self) -> None:
        if self.busy.is_set():
            return
        self.busy.set()
        self.set_action_buttons_enabled(False)
        self.live_paused.clear()
        self.live_generation += 1
        generation = self.live_generation
        self.link.status_mode = "auto"
        self.pause_button_var.set("Pause Live Feed")
        self.connection_var.set("reconnecting")
        self.connection_label.configure(style="Warn.TLabel")

        def worker() -> None:
            try:
                self.link.close()
                status = self.link.read_status()
                self.queue.put(("operation_done", "reconnect complete"))
                self.queue.put(("status", (generation, status)))
            except Exception as exc:
                self.queue.put(("operation_done", "reconnect failed"))
                self.queue.put(("disconnected", (generation, exc)))

        threading.Thread(target=worker, daemon=True).start()

    def flash_flight(self) -> None:
        if not self.begin_exclusive_operation("flashing flight logger"):
            return
        threading.Thread(target=self._flash_flight_worker, daemon=True).start()

    def _flash_flight_worker(self) -> None:
        done_message = "flight flash finished"
        try:
            rc = run_process([PIO, "run", "-e", FLIGHT_ENV, "-t", "upload"], FIRMWARE_ROOT, self.thread_log)
            if rc != 0:
                self.queue.put(("error", f"Flight flash failed rc={rc}"))
                done_message = "flight flash failed"
            else:
                self.link.status_mode = "auto"
                self.queue.put(("message", "Flight logger flashed."))
        finally:
            self.queue.put(("operation_done", done_message))

    def wipe_restore(self) -> None:
        if not self.begin_exclusive_operation("wiping flash then restoring flight logger"):
            return
        self.log("wipe sequence: flash erase image -> wait for erase/init -> flash flight image")
        threading.Thread(target=self._wipe_restore_worker, daemon=True).start()

    def _wipe_restore_worker(self) -> None:
        done_message = "wipe + restore finished"
        try:
            rc = run_process([PIO, "run", "-e", ERASE_ENV, "-t", "upload"], FIRMWARE_ROOT, self.thread_log)
            if rc != 0:
                self.queue.put(("error", f"Erase image flash failed rc={rc}"))
                done_message = "wipe + restore failed"
                return
            self.thread_log("erase image flashed; waiting for logger to reinitialize")
            erase_ok = self.wait_for_erase_image(timeout_s=900.0)
            self.link.close()
            if not erase_ok:
                self.queue.put(("error", "Erase image did not report clean logger init before timeout."))
                done_message = "wipe + restore failed"
                return
            rc = run_process([PIO, "run", "-e", FLIGHT_ENV, "-t", "upload"], FIRMWARE_ROOT, self.thread_log)
            if rc != 0:
                self.queue.put(("error", f"Flight restore failed rc={rc}; erase image may still be installed."))
                done_message = "wipe + restore failed; erase image may still be installed"
            else:
                self.link.status_mode = "auto"
                self.queue.put(("message", "Flash wiped and flight logger restored."))
        finally:
            self.queue.put(("operation_done", done_message))

    def wait_for_erase_image(self, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                status = self.link.read_status()
                if (
                    status.get("magic") == STATUS_MAGIC
                    and status.get("firmware_flavor") == 2
                    and status.get("logger_ok") == 1
                    and status.get("boot_logged") == 1
                    and status.get("logger_status") == 0
                ):
                    return True
            except Exception as exc:
                self.thread_log(f"waiting for erase image: {exc}")
                self.link.close()
            time.sleep(2.0)
        return False

    def retrieve_flash(self) -> None:
        if not self.begin_exclusive_operation("flash readout running"):
            return
        self.log("flash readout started; live plots frozen")
        threading.Thread(target=self._retrieve_flash_worker, daemon=True).start()

    def load_saved_run(self) -> None:
        RUNS_ROOT.mkdir(parents=True, exist_ok=True)
        selected = filedialog.askdirectory(
            parent=self.root,
            title="Load Mu saved run",
            initialdir=str(RUNS_ROOT),
        )
        if not selected:
            return
        run_dir = Path(selected)
        try:
            parsed, dump = load_run_bundle(run_dir, self.log)
        except Exception as exc:
            self.log(f"ERROR: failed to load {run_dir}: {exc}")
            return
        self.apply_readout(parsed, dump, run_dir)

    def _retrieve_flash_worker(self) -> None:
        done_message = "flash readout finished"
        try:
            self.link.close()
            self.link.status_mode = "auto"
            status = self.link.read_status()
            label, severity = firmware_label(status)
            if severity == "bad" or status.get("version", 0) < 2:
                raise RuntimeError(f"firmware does not support mailbox readout: {label}")
            mailbox = self.link.read_mailbox_header()
            if mailbox.get("magic") != MAILBOX_MAGIC:
                raise RuntimeError("flash mailbox magic missing")
            log_start = status.get("log_start") or mailbox.get("log_start") or 0
            log_length = status.get("log_length") or mailbox.get("log_length") or 0
            used = status.get("log_used_bytes", 0)
            if status.get("log_full", 0):
                used = log_length
            if used <= 0 or used > log_length:
                used = log_length
            if used <= 0:
                raise RuntimeError("log length is zero")
            self.thread_log(f"reading flash log start=0x{log_start:06x} bytes={used}")
            dump = bytearray()
            address = log_start
            remaining = used
            next_progress = 0.0
            while remaining > 0:
                chunk_len = min(512, remaining)
                dump.extend(self.link.read_flash_chunk(address, chunk_len))
                address += chunk_len
                remaining -= chunk_len
                done = len(dump) / used
                if done >= next_progress or remaining == 0:
                    self.thread_log(f"readout {done * 100.0:.1f}% ({len(dump)}/{used} B)")
                    next_progress += 0.05
            parsed = parse_flash_dump(bytes(dump), self.thread_log)
            out = save_run_bundle(parsed, bytes(dump))
            self.queue.put(("readout", (parsed, bytes(dump), out)))
        except Exception as exc:
            self.queue.put(("error", str(exc)))
            done_message = "flash readout failed"
        finally:
            self.queue.put(("operation_done", done_message))

    def apply_readout(self, parsed: dict[str, object], _dump: bytes, out: Path) -> None:
        self.current_readout = parsed
        summary = parsed["summary"]  # type: ignore[index]
        warnings = summary.get("warnings", [])  # type: ignore[union-attr]
        self.readout_summary_var.set(
            "Run: {}\nrun={} records={} events={} pressure={} duration={:.1f}s rate={:.1f}/min used={} B warnings={}".format(
                out,
                summary.get("latest_run_id", 0),  # type: ignore[union-attr]
                summary.get("records", 0),  # type: ignore[union-attr]
                summary.get("event_records", 0),  # type: ignore[union-attr]
                summary.get("pressure_records", 0),  # type: ignore[union-attr]
                summary.get("duration_s", 0.0),  # type: ignore[union-attr]
                summary.get("event_rate_per_min", 0.0),  # type: ignore[union-attr]
                summary.get("used_bytes", 0),  # type: ignore[union-attr]
                len(warnings),
            )
        )
        self.event_list.delete(0, tk.END)
        self.waveform_plot.set_data("Event Waveform", "mV", [])
        for event in parsed["events"]:  # type: ignore[index,union-attr]
            self.event_list.insert(
                tk.END,
                "seq {:04d} t={:.2f}s amp={}mV p={}Pa".format(
                    int(event.get("event_seq", 0)),
                    int(event.get("timestamp_ms", 0)) / 1000.0,
                    event.get("amplitude_mv", "?"),
                    event.get("pressure_pa", "?"),
                ),
            )
        self.redraw_experiment_plot()
        if self.event_list.size() > 0:
            self.event_list.selection_set(0)
            self.on_event_selected(None)
        self.log(f"readout loaded from {out}")

    def redraw_experiment_plot(self) -> None:
        if self.current_readout is None:
            return
        events = self.current_readout["events"]  # type: ignore[index]
        pressures = self.current_readout["pressures"]  # type: ignore[index]
        event_x = [int(event.get("timestamp_ms", 0)) / 1000.0 / 60.0 for event in events]
        event_y = list(range(1, len(event_x) + 1))
        pressure_x = [int(row.get("timestamp_ms", 0)) / 1000.0 / 60.0 for row in pressures]
        pressure_y = [float(row.get("pressure_pa", 0)) / 1000.0 for row in pressures]
        pressure_height_y = [float(row.get("pressure_height_m", 0.0)) for row in pressures]
        self.experiment_time_plot.set_data(
            "Flight Profile + Particle Count vs Time",
            "events",
            "height m / pressure kPa",
            [("cumulative events", event_x, event_y, "#65d46e")],
            [
                ("pressure kPa", pressure_x, pressure_y, "#b892ff"),
                ("baro height m", pressure_x, pressure_height_y, "#ff7f6e"),
            ],
        )

        height_bins, rate_bins = self.event_rate_by_height(events, pressures)
        self.experiment_correlation_plot.set_data(
            "Event Rate vs Baro Height",
            "events/min",
            [("rate by height bin", height_bins, rate_bins, "#56a8ff")],
        )

    @staticmethod
    def event_rate_by_height(
        events: list[dict[str, object]],
        pressures: list[dict[str, object]],
    ) -> tuple[list[float], list[float]]:
        if len(pressures) < 2:
            return [], []

        samples = [
            (
                int(row.get("timestamp_ms", 0)),
                float(row.get("pressure_height_m", 0.0)),
            )
            for row in pressures
        ]
        samples.sort(key=lambda item: item[0])
        heights = [height for _timestamp, height in samples]
        min_height = min(heights)
        max_height = max(heights)
        span = max_height - min_height
        if span <= 0.0:
            return [], []

        bin_count = max(4, min(24, int(span / 25.0) + 1))
        bin_width = span / bin_count
        dwell_s = [0.0 for _ in range(bin_count)]
        counts = [0 for _ in range(bin_count)]

        def bin_index(height: float) -> int:
            idx = int((height - min_height) / bin_width)
            return max(0, min(bin_count - 1, idx))

        for (t0, h0), (t1, _h1) in zip(samples, samples[1:]):
            dt_s = max(0.0, (t1 - t0) / 1000.0)
            dwell_s[bin_index(h0)] += dt_s

        for event in events:
            height = float(event.get("pressure_height_m", 0.0))
            counts[bin_index(height)] += 1

        centers = [min_height + (i + 0.5) * bin_width for i in range(bin_count)]
        rates = [
            (counts[i] / dwell_s[i] * 60.0) if dwell_s[i] > 0.0 else 0.0
            for i in range(bin_count)
        ]
        return centers, rates

    def on_event_selected(self, _event: object) -> None:
        if self.current_readout is None:
            return
        selection = self.event_list.curselection()
        if not selection:
            return
        event = self.current_readout["events"][selection[0]]  # type: ignore[index]
        samples = event.get("samples", [])
        if not isinstance(samples, list) or not samples:
            return
        pre = int(event.get("waveform_pre_samples", 16))
        sample_interval_us = int(event.get("sample_interval_us", 10))
        xs = [(i - pre + 1) * sample_interval_us for i in range(len(samples))]
        ys = [raw_to_mv(int(raw)) for raw in samples]
        baseline = float(event.get("baseline_mv", ys[0]))
        baseline_y = [baseline for _ in xs]
        self.waveform_plot.set_data(
            f"Event {event.get('event_seq', '?')} waveform",
            "mV",
            [
                ("catch", xs, ys, "#56a8ff"),
                ("baseline", xs, baseline_y, "#65d46e"),
            ],
        )

    def on_close(self) -> None:
        self.stop_event.set()
        self.link.close()
        self.root.destroy()


def self_test() -> None:
    assert HEADER_STRUCT.size == 40
    assert BOOT_STRUCT.size == 60
    assert PRESSURE_STRUCT.size == 28
    assert EVENT_PREFIX_STRUCT.size == 76
    assert align4(40 + 204) == 244
    assert align4(40 + 28) == 68
    assert align4(40 + 60) == 100
    print("mu_dashboard self-test passed")


def main() -> int:
    if "--self-test" in sys.argv:
        self_test()
        return 0
    RUNS_ROOT.mkdir(parents=True, exist_ok=True)
    root = tk.Tk()
    DashboardApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
