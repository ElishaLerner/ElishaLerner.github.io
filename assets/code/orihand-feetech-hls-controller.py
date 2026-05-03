#!/usr/bin/env python3
"""
Standalone Feetech HLS controller.

This single file combines:
- serial bus communication
- a desktop GUI for daily use
- CLI commands for troubleshooting

Typical use:
    python feetech_hls_controller.py

CLI examples:
    python feetech_hls_controller.py ports
    python feetech_hls_controller.py --port COM7 scan --start 0 --end 10
    python feetech_hls_controller.py --port COM7 read --id 1
"""

from __future__ import annotations

import argparse
import dataclasses
import re
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Iterable, Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise SystemExit(
        "This script needs pyserial. Install it with: pip install pyserial"
    ) from exc


HEADER = b"\xFF\xFF"


class Inst:
    PING = 0x01
    READ = 0x02
    WRITE = 0x03
    REG_WRITE = 0x04
    ACTION = 0x05
    RESET = 0x06


class HLSReg:
    ID = 5
    MODE = 33
    TORQUE_ENABLE = 40
    ACC = 41
    GOAL_TORQUE_L = 44
    PRESENT_POSITION_L = 56
    PRESENT_SPEED_L = 58
    PRESENT_LOAD_L = 60
    PRESENT_VOLTAGE = 62
    PRESENT_TEMPERATURE = 63
    MOVING = 66
    PRESENT_CURRENT_L = 69


class HLSMode:
    SERVO = 0
    SPEED = 1
    CURRENT = 2


DEFAULT_BAUDS = ("1000000", "115200")
DEFAULT_SCAN_END = "10"
DEFAULT_POSITION_SPEED_RPM = "30"
DEFAULT_CONTINUOUS_SPEED_RPM = "15"
DEFAULT_ACCEL_DEG_S2 = "90"
DEFAULT_CURRENT_LIMIT_A = "0.45"
DEFAULT_CURRENT_RAW = "20"
DEFAULT_JOG_STEP_DEG = "10"
DEFAULT_ALL_FINGERS_TARGET_DEG = "180.0"

SOFT_HOME_SPEED_RPM = 8.0
SOFT_HOME_ACCEL_DEG_S2 = 30.0
SOFT_HOME_CURRENT_LIMIT_A = 0.25

SERVO_MIN_POS = 0
SERVO_MAX_POS = 4095
SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 195.0

RAW_POSITION_TO_DEG = 360.0 / 4096.0
RAW_SPEED_TO_RPM = 0.732
RAW_ACCEL_TO_DEG_S2 = 8.7
MAX_RAW_SPEED = 136
MAX_RAW_ACCEL = 255
MAX_RAW_CURRENT_LIMIT = 1000
MAX_MOTOR_CURRENT_A = 1.5
FINGER_NAME_BY_ID = {
    1: "Thumb",
    2: "Index",
    3: "Middle",
}
SEQUENCE_STORAGE_DIR = Path(__file__).resolve().parent / "SavedSequences"


@dataclasses.dataclass
class StatusPacket:
    servo_id: int
    error: int
    params: bytes


@dataclasses.dataclass
class Feedback:
    position: int
    speed: int
    load: int
    voltage_v: float
    temperature_c: int
    moving: bool
    current_raw: int


@dataclasses.dataclass
class ServoPanelState:
    servo_id: int
    frame: ttk.LabelFrame
    position_entry: ttk.Entry
    advanced_frame: ttk.LabelFrame
    advanced_toggle_var: tk.StringVar
    live_position_var: tk.StringVar
    position_var: tk.StringVar
    jog_step_var: tk.StringVar
    position_speed_rpm_var: tk.StringVar
    continuous_speed_rpm_var: tk.StringVar
    accel_deg_s2_var: tk.StringVar
    current_limit_a_var: tk.StringVar
    current_var: tk.StringVar
    move_speed_guide_var: tk.StringVar
    accel_guide_var: tk.StringVar
    current_limit_guide_var: tk.StringVar
    continuous_speed_guide_var: tk.StringVar
    feedback_var: tk.StringVar
    last_control_mode: str


def checksum(payload: Iterable[int]) -> int:
    return (~(sum(payload) & 0xFF)) & 0xFF


def le_u16(value: int) -> bytes:
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def encode_signed_15(value: int) -> int:
    if value < 0:
        return ((-value) & 0x7FFF) | 0x8000
    return value & 0x7FFF


def decode_signed_15(value: int) -> int:
    if value & 0x8000:
        return -(value & 0x7FFF)
    return value


def decode_signed_10(value: int) -> int:
    if value & (1 << 10):
        return -(value & ~(1 << 10))
    return value


def u16_from(buf: bytes, offset: int) -> int:
    return buf[offset] | (buf[offset + 1] << 8)


def list_ports(return_devices_only: bool = False) -> list[str] | None:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        if return_devices_only:
            return []
        print("No serial ports found.")
        return None
    if return_devices_only:
        return [port.device for port in ports]
    for port in ports:
        print(f"{port.device:12s} {port.description}")
    return None


def finger_name_for_id(servo_id: int) -> str:
    return FINGER_NAME_BY_ID.get(servo_id, f"Finger {servo_id}")


def finger_label_for_id(servo_id: int) -> str:
    return f"{finger_name_for_id(servo_id)} (ID {servo_id})"


def sanitize_sequence_name(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9 _-]+", "", name).strip()
    cleaned = cleaned.replace(" ", "_")
    return cleaned or "sequence"


class FeetechBus:
    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 0.1):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.timeout = timeout

    def close(self) -> None:
        self.ser.close()

    def __enter__(self) -> "FeetechBus":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _build_packet(self, servo_id: int, instruction: int, params: bytes = b"") -> bytes:
        length = len(params) + 2
        body = bytes((servo_id, length, instruction)) + params
        return HEADER + body + bytes((checksum(body),))

    def _read_status(self, expected_id: Optional[int]) -> StatusPacket:
        deadline = time.monotonic() + self.timeout
        buf = bytearray()

        while time.monotonic() < deadline:
            chunk = self.ser.read(1)
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= 4:
                header_index = buf.find(HEADER)
                if header_index < 0:
                    buf.clear()
                    break
                if header_index > 0:
                    del buf[:header_index]
                if len(buf) < 4:
                    break

                servo_id = buf[2]
                length = buf[3]
                packet_len = length + 4
                if len(buf) < packet_len:
                    break

                packet = bytes(buf[:packet_len])
                del buf[:packet_len]

                payload = packet[2:-1]
                received_checksum = packet[-1]
                if checksum(payload) != received_checksum:
                    continue

                if expected_id is not None and servo_id != expected_id:
                    continue

                return StatusPacket(servo_id=servo_id, error=packet[4], params=packet[5:-1])

        raise TimeoutError("Timed out waiting for servo response")

    def transact(
        self,
        servo_id: int,
        instruction: int,
        params: bytes = b"",
        expect_reply: bool = True,
    ) -> Optional[StatusPacket]:
        self.ser.reset_input_buffer()
        self.ser.write(self._build_packet(servo_id, instruction, params))
        self.ser.flush()
        if not expect_reply or servo_id == 0xFE:
            return None
        return self._read_status(expected_id=servo_id)

    def ping(self, servo_id: int) -> bool:
        self.transact(servo_id, Inst.PING)
        return True

    def read(self, servo_id: int, address: int, size: int) -> bytes:
        status = self.transact(servo_id, Inst.READ, bytes((address, size)))
        if status is None:
            raise RuntimeError("READ unexpectedly returned no status packet")
        if status.error:
            raise RuntimeError(f"Servo {servo_id} returned error code 0x{status.error:02X}")
        if len(status.params) != size:
            raise RuntimeError(
                f"Expected {size} data bytes from servo {servo_id}, got {len(status.params)}"
            )
        return status.params

    def write(self, servo_id: int, address: int, data: bytes, reg_write: bool = False) -> None:
        instruction = Inst.REG_WRITE if reg_write else Inst.WRITE
        status = self.transact(servo_id, instruction, bytes((address,)) + data)
        if status is not None and status.error:
            raise RuntimeError(f"Servo {servo_id} returned error code 0x{status.error:02X}")

    def write_byte(self, servo_id: int, address: int, value: int) -> None:
        self.write(servo_id, address, bytes((value & 0xFF,)))

    def write_word(self, servo_id: int, address: int, value: int) -> None:
        self.write(servo_id, address, le_u16(value))

    def action(self) -> None:
        self.transact(0xFE, Inst.ACTION, expect_reply=False)

    def set_mode(self, servo_id: int, mode: int) -> None:
        self.write_byte(servo_id, HLSReg.MODE, mode)

    def set_id(self, servo_id: int, new_id: int) -> None:
        self.write_byte(servo_id, HLSReg.ID, new_id)

    def enable_torque(self, servo_id: int, enabled: bool) -> None:
        self.write_byte(servo_id, HLSReg.TORQUE_ENABLE, 1 if enabled else 0)

    def set_position(
        self,
        servo_id: int,
        position: int,
        speed: int = 0,
        acc: int = 0,
        torque_limit_raw: int = 500,
        reg_write: bool = False,
    ) -> None:
        payload = bytes((acc & 0xFF,)) + le_u16(encode_signed_15(position))
        payload += le_u16(torque_limit_raw & 0xFFFF)
        payload += le_u16(speed & 0xFFFF)
        self.write(servo_id, HLSReg.ACC, payload, reg_write=reg_write)

    def set_speed(
        self,
        servo_id: int,
        speed: int,
        acc: int = 0,
        torque_limit_raw: int = 500,
    ) -> None:
        payload = bytes((acc & 0xFF,)) + le_u16(0)
        payload += le_u16(torque_limit_raw & 0xFFFF)
        payload += le_u16(encode_signed_15(speed))
        self.write(servo_id, HLSReg.ACC, payload)

    def set_current_raw(self, servo_id: int, target_current_raw: int) -> None:
        self.write_word(servo_id, HLSReg.GOAL_TORQUE_L, encode_signed_15(target_current_raw))

    def read_feedback(self, servo_id: int) -> Feedback:
        raw = self.read(servo_id, HLSReg.PRESENT_POSITION_L, 15)
        return Feedback(
            position=decode_signed_15(u16_from(raw, 0)),
            speed=decode_signed_15(u16_from(raw, 2)),
            load=decode_signed_10(u16_from(raw, 4)),
            voltage_v=raw[6] / 10.0,
            temperature_c=raw[7],
            moving=bool(raw[10]),
            current_raw=decode_signed_15(u16_from(raw, 13)),
        )

    def scan(self, start_id: int = 0, end_id: int = 20) -> list[int]:
        found: list[int] = []
        for servo_id in range(start_id, end_id + 1):
            try:
                self.ping(servo_id)
            except TimeoutError:
                continue
            found.append(servo_id)
        return found


class ServoControlApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Feetech HLS Controller")
        self.root.geometry("1180x760")

        self.bus: FeetechBus | None = None
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=DEFAULT_BAUDS[0])
        self.scan_start_var = tk.StringVar(value="0")
        self.scan_end_var = tk.StringVar(value=DEFAULT_SCAN_END)
        self.status_var = tk.StringVar(value="Disconnected")
        self.auto_refresh_var = tk.BooleanVar(value=True)
        self.refresh_ms_var = tk.StringVar(value="500")
        self.all_target_deg_var = tk.StringVar(value=DEFAULT_ALL_FINGERS_TARGET_DEG)
        self.all_jog_step_deg_var = tk.StringVar(value=DEFAULT_JOG_STEP_DEG)
        self.all_move_speed_rpm_var = tk.StringVar(value=DEFAULT_POSITION_SPEED_RPM)
        self.all_accel_deg_s2_var = tk.StringVar(value=DEFAULT_ACCEL_DEG_S2)
        self.all_current_limit_a_var = tk.StringVar(value=DEFAULT_CURRENT_LIMIT_A)
        self.all_guide_var = tk.StringVar(
            value="Hand-safe range: 0 to 195 deg. Home = 180 deg. Close = 0 deg."
        )
        self.motion_planner_toggle_var = tk.StringVar(value="Show Sequence Planner")
        self.motion_plan_bend_limit_deg_var = tk.StringVar(value="20.0")
        self.motion_plan_status_var = tk.StringVar(
            value="Sequence planner supports wait, home, and go-to steps. Commands are clamped to the bend limit and hand-safe range."
        )
        self.motion_plan_state: dict | None = None
        self.motion_plan_after_id: str | None = None
        self.motion_planner_frame: ttk.LabelFrame | None = None
        self.motion_plan_text: tk.Text | None = None
        self.sequence_name_var = tk.StringVar(value="grasp_sequence")
        self.sequence_saved_var = tk.StringVar(
            value=f"Saved sequences folder: {SEQUENCE_STORAGE_DIR}"
        )
        self.sequence_selector: ttk.Combobox | None = None
        self.single_id_current_var = tk.StringVar(value="-")
        self.single_id_new_var = tk.StringVar(value="2")

        self.servo_panels: dict[int, ServoPanelState] = {}

        self.ensure_sequence_storage_dir()
        self._build_ui()
        self.refresh_ports()
        self._schedule_refresh()

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        top = ttk.Frame(self.root, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(8, weight=1)

        ttk.Label(top, text="Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=20, state="readonly")
        self.port_combo.grid(row=1, column=0, sticky="ew", padx=(0, 8))

        ttk.Button(top, text="Refresh Ports", command=self.refresh_ports).grid(
            row=1, column=1, sticky="ew", padx=(0, 12)
        )

        ttk.Label(top, text="Baud").grid(row=0, column=2, sticky="w")
        self.baud_combo = ttk.Combobox(
            top, textvariable=self.baud_var, values=DEFAULT_BAUDS, width=12, state="readonly"
        )
        self.baud_combo.grid(row=1, column=2, sticky="ew", padx=(0, 12))

        ttk.Button(top, text="Connect", command=self.connect).grid(row=1, column=3, sticky="ew", padx=(0, 6))
        ttk.Button(top, text="Disconnect", command=self.disconnect).grid(row=1, column=4, sticky="ew", padx=(0, 12))

        ttk.Label(top, text="Scan IDs").grid(row=0, column=5, sticky="w")
        scan_frame = ttk.Frame(top)
        scan_frame.grid(row=1, column=5, sticky="w", padx=(0, 12))
        ttk.Entry(scan_frame, textvariable=self.scan_start_var, width=5).grid(row=0, column=0)
        ttk.Label(scan_frame, text="to").grid(row=0, column=1, padx=4)
        ttk.Entry(scan_frame, textvariable=self.scan_end_var, width=5).grid(row=0, column=2)

        ttk.Button(top, text="Scan", command=self.scan).grid(row=1, column=6, sticky="ew", padx=(0, 12))
        ttk.Checkbutton(top, text="Auto Refresh", variable=self.auto_refresh_var).grid(
            row=1, column=7, sticky="w"
        )

        refresh_frame = ttk.Frame(top)
        refresh_frame.grid(row=1, column=8, sticky="e")
        ttk.Label(refresh_frame, text="Refresh ms").grid(row=0, column=0, padx=(0, 6))
        ttk.Entry(refresh_frame, textvariable=self.refresh_ms_var, width=6).grid(row=0, column=1)

        ttk.Label(self.root, textvariable=self.status_var, padding=(10, 0, 10, 10)).grid(
            row=2, column=0, sticky="ew"
        )

        canvas_frame = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        canvas_frame.grid(row=1, column=0, sticky="nsew")
        canvas_frame.columnconfigure(0, weight=1)
        canvas_frame.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(canvas_frame, highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(canvas_frame, orient="vertical", command=self.canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.canvas.configure(yscrollcommand=scrollbar.set)

        self.panels_container = ttk.Frame(self.canvas)
        self.panels_container.columnconfigure(0, weight=1)
        self.canvas_window = self.canvas.create_window((0, 0), window=self.panels_container, anchor="nw")
        self._build_all_fingers_panel()

        self.panels_container.bind("<Configure>", self._on_container_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

    def ensure_sequence_storage_dir(self) -> None:
        SEQUENCE_STORAGE_DIR.mkdir(parents=True, exist_ok=True)

    def sequence_file_path(self, name: str) -> Path:
        safe_name = sanitize_sequence_name(name)
        return SEQUENCE_STORAGE_DIR / f"{safe_name}.txt"

    def refresh_saved_sequences(self) -> None:
        self.ensure_sequence_storage_dir()
        names = sorted(path.stem for path in SEQUENCE_STORAGE_DIR.glob("*.txt"))
        if self.sequence_selector is not None:
            self.sequence_selector["values"] = names
        if names and self.sequence_name_var.get().strip() not in names:
            self.sequence_name_var.set(names[0])
        self.sequence_saved_var.set(f"Saved sequences: {len(names)}")

    def save_current_sequence(self) -> None:
        if self.motion_plan_text is None:
            return

        name = self.sequence_name_var.get().strip()
        safe_name = sanitize_sequence_name(name)
        path = self.sequence_file_path(name)
        script = self.motion_plan_text.get("1.0", tk.END).rstrip() + "\n"

        try:
            self.ensure_sequence_storage_dir()
            path.write_text(script, encoding="utf-8")
        except Exception as exc:
            messagebox.showerror("Save Failed", str(exc))
            self.sequence_saved_var.set("Save failed.")
            return

        self.sequence_name_var.set(safe_name)
        self.sequence_saved_var.set(f"Saved: {path.name}")
        self.refresh_saved_sequences()

    def load_saved_sequence(self) -> None:
        if self.motion_plan_text is None:
            return

        name = self.sequence_name_var.get().strip()
        if not name:
            messagebox.showwarning("No Sequence Name", "Choose or enter a saved sequence name first.")
            return

        path = self.sequence_file_path(name)
        if not path.exists():
            messagebox.showwarning("Missing Sequence", f"No saved sequence named '{sanitize_sequence_name(name)}' was found.")
            self.sequence_saved_var.set("Selected sequence was not found.")
            return

        try:
            script = path.read_text(encoding="utf-8")
        except Exception as exc:
            messagebox.showerror("Load Failed", str(exc))
            self.sequence_saved_var.set("Load failed.")
            return

        self.motion_plan_text.delete("1.0", tk.END)
        self.motion_plan_text.insert("1.0", script)
        self.sequence_name_var.set(path.stem)
        self.sequence_saved_var.set(f"Loaded: {path.name}")
        self.motion_plan_text.focus_set()

    def _build_all_fingers_panel(self) -> None:
        frame = ttk.LabelFrame(self.panels_container, text="All Fingers", padding=10)
        frame.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        for col in range(8):
            frame.columnconfigure(col, weight=1 if col in (1, 3, 5, 7) else 0)

        ttk.Label(frame, text="Target (deg)").grid(row=0, column=0, sticky="w")
        self.all_target_entry = ttk.Entry(frame, textvariable=self.all_target_deg_var, width=10)
        self.all_target_entry.grid(row=0, column=1, sticky="ew", padx=(0, 10))
        self.all_target_entry.bind("<Return>", lambda _event: self.move_all_fingers())
        self.all_target_entry.bind("<FocusIn>", lambda _event: self.select_entry_text(self.all_target_entry))

        ttk.Label(frame, text="Jog Step (deg)").grid(row=0, column=2, sticky="w")
        ttk.Entry(frame, textvariable=self.all_jog_step_deg_var, width=10).grid(
            row=0, column=3, sticky="ew", padx=(0, 10)
        )

        ttk.Label(frame, text="Move Speed (RPM)").grid(row=0, column=4, sticky="w")
        ttk.Entry(frame, textvariable=self.all_move_speed_rpm_var, width=10).grid(
            row=0, column=5, sticky="ew", padx=(0, 10)
        )

        ttk.Label(frame, text="Current Limit (A)").grid(row=0, column=6, sticky="w")
        ttk.Entry(frame, textvariable=self.all_current_limit_a_var, width=10).grid(
            row=0, column=7, sticky="ew"
        )

        ttk.Label(frame, text="Accel (deg/s^2)").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=self.all_accel_deg_s2_var, width=10).grid(
            row=1, column=1, sticky="ew", padx=(0, 10), pady=(8, 0)
        )
        ttk.Label(frame, textvariable=self.all_guide_var).grid(
            row=1, column=2, columnspan=6, sticky="w", pady=(8, 0)
        )

        buttons = ttk.Frame(frame)
        buttons.grid(row=2, column=0, columnspan=8, sticky="ew", pady=(10, 0))
        for idx in range(8):
            buttons.columnconfigure(idx, weight=1)

        ttk.Button(buttons, text="Refresh All", command=self.refresh_all_feedback).grid(
            row=0, column=0, sticky="ew", padx=(0, 6)
        )
        ttk.Button(buttons, text="Open (180 deg)", command=lambda: self.move_all_to_degrees(180.0)).grid(
            row=0, column=1, sticky="ew", padx=(0, 6)
        )
        ttk.Button(buttons, text="Close (0 deg)", command=lambda: self.move_all_to_degrees(0.0)).grid(
            row=0, column=2, sticky="ew", padx=(0, 6)
        )
        ttk.Button(buttons, text="Jog -", command=lambda: self.jog_all_fingers(-1)).grid(
            row=0, column=3, sticky="ew", padx=(0, 6)
        )
        ttk.Button(buttons, text="Jog +", command=lambda: self.jog_all_fingers(1)).grid(
            row=0, column=4, sticky="ew", padx=(0, 6)
        )
        ttk.Button(buttons, text="Move All", command=self.move_all_fingers).grid(
            row=0, column=5, sticky="ew", padx=(0, 6)
        )
        ttk.Button(buttons, text="Torque On All", command=lambda: self.set_torque_all(True)).grid(
            row=0, column=6, sticky="ew", padx=(0, 6)
        )
        ttk.Button(buttons, text="Torque Off All", command=lambda: self.set_torque_all(False)).grid(
            row=0, column=7, sticky="ew"
        )

        self.single_id_frame = ttk.LabelFrame(frame, text="Single Servo ID Setup", padding=10)
        self.single_id_frame.grid(row=3, column=0, columnspan=8, sticky="ew", pady=(10, 0))
        for col in range(6):
            self.single_id_frame.columnconfigure(col, weight=1 if col in (1, 3, 5) else 0)

        ttk.Label(self.single_id_frame, text="Current ID").grid(row=0, column=0, sticky="w")
        ttk.Label(self.single_id_frame, textvariable=self.single_id_current_var).grid(
            row=0, column=1, sticky="w", padx=(0, 10)
        )
        ttk.Label(self.single_id_frame, text="New ID").grid(row=0, column=2, sticky="w")
        ttk.Entry(self.single_id_frame, textvariable=self.single_id_new_var, width=8).grid(
            row=0, column=3, sticky="ew", padx=(0, 10)
        )
        ttk.Label(
            self.single_id_frame,
            text="Visible only when exactly one servo is discovered. Use this before daisy-chaining all three.",
        ).grid(row=0, column=4, sticky="w", padx=(0, 10))
        ttk.Button(self.single_id_frame, text="Set ID", command=self.set_single_servo_id).grid(
            row=0, column=5, sticky="ew"
        )
        self.single_id_frame.grid_remove()

        planner_toggle_row = ttk.Frame(frame)
        planner_toggle_row.grid(row=4, column=0, columnspan=8, sticky="ew", pady=(10, 0))
        planner_toggle_row.columnconfigure(0, weight=1)
        planner_toggle_row.columnconfigure(1, weight=0)
        ttk.Label(
            planner_toggle_row,
            text="Sequence planner runs timed step-by-step hand motions with waits, targets, and current limits.",
        ).grid(row=0, column=0, sticky="w")
        ttk.Button(
            planner_toggle_row,
            textvariable=self.motion_planner_toggle_var,
            command=self.toggle_motion_planner,
        ).grid(row=0, column=1, sticky="e")

        self.motion_planner_frame = ttk.LabelFrame(frame, text="Sequence Planner", padding=10)
        self.motion_planner_frame.grid(row=5, column=0, columnspan=8, sticky="ew", pady=(10, 0))
        self.motion_planner_frame.columnconfigure(0, weight=1)
        self.motion_planner_frame.columnconfigure(1, weight=0)

        settings_row = ttk.Frame(self.motion_planner_frame)
        settings_row.grid(row=0, column=0, columnspan=2, sticky="ew")
        for col in range(6):
            settings_row.columnconfigure(col, weight=1 if col in (1, 3, 5) else 0)

        ttk.Label(settings_row, text="Most Closed (deg)").grid(row=0, column=0, sticky="w")
        ttk.Entry(settings_row, textvariable=self.motion_plan_bend_limit_deg_var, width=10).grid(
            row=0, column=1, sticky="ew", padx=(0, 10)
        )
        ttk.Label(settings_row, text="Move Speed (RPM)").grid(row=0, column=2, sticky="w")
        ttk.Entry(settings_row, textvariable=self.all_move_speed_rpm_var, width=10).grid(
            row=0, column=3, sticky="ew", padx=(0, 10)
        )
        ttk.Label(settings_row, text="Accel (deg/s^2)").grid(row=0, column=4, sticky="w")
        ttk.Entry(settings_row, textvariable=self.all_accel_deg_s2_var, width=10).grid(
            row=0, column=5, sticky="ew"
        )

        sequence_help = (
            "Available commands:\n"
            "home\n"
            "wait <seconds>\n"
            "go to <degrees> with a current limit of <amps> A\n\n"
            "Modifier structure:\n"
            "<finger target>: <command>\n\n"
            "Valid finger targets:\n"
            "thumb, index, middle, all, hand, fingers, id <number>, finger <number>\n\n"
            "Examples:\n"
            "wait 1.5 seconds\n"
            "go to 90 degrees with a current limit of 0.45 A\n"
            "thumb: go to 120 degrees with a current limit of 0.35 A\n"
            "index: home\n"
            "middle: wait 0.5 seconds"
        )
        ttk.Label(
            self.motion_planner_frame,
            text=sequence_help,
            justify="left",
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(10, 0))

        saved_row = ttk.Frame(self.motion_planner_frame)
        saved_row.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        for col in range(6):
            saved_row.columnconfigure(col, weight=1 if col in (1, 3) else 0)

        ttk.Label(saved_row, text="Sequence Name").grid(row=0, column=0, sticky="w")
        self.sequence_selector = ttk.Combobox(saved_row, textvariable=self.sequence_name_var, width=26)
        self.sequence_selector.grid(row=0, column=1, sticky="ew", padx=(0, 10))
        ttk.Button(saved_row, text="Refresh Saved", command=self.refresh_saved_sequences).grid(
            row=0, column=2, sticky="ew", padx=(0, 6)
        )
        ttk.Button(saved_row, text="Load Saved", command=self.load_saved_sequence).grid(
            row=0, column=3, sticky="ew", padx=(0, 6)
        )
        ttk.Button(saved_row, text="Save Current", command=self.save_current_sequence).grid(
            row=0, column=4, sticky="ew", padx=(0, 6)
        )
        ttk.Label(saved_row, textvariable=self.sequence_saved_var).grid(
            row=0, column=5, sticky="w"
        )

        self.motion_plan_text = tk.Text(self.motion_planner_frame, height=9, wrap="word")
        self.motion_plan_text.grid(row=3, column=0, sticky="nsew", pady=(10, 0))
        self.motion_plan_text.insert(
            "1.0",
            "# Sequence examples\n"
            "wait 1.5 seconds\n"
            "go to 90 degrees with a current limit of 0.45 A\n"
            "go to 60 degrees with a current limit of 0.70 A\n"
            "wait 5 seconds\n"
            "home\n\n"
            "# Optional finger-specific steps\n"
            "thumb: go to 120 degrees with a current limit of 0.35 A\n"
            "index: home\n"
            "middle: wait 0.5 seconds\n",
        )
        planner_scroll = ttk.Scrollbar(self.motion_planner_frame, orient="vertical", command=self.motion_plan_text.yview)
        planner_scroll.grid(row=3, column=1, sticky="ns", pady=(10, 0))
        self.motion_plan_text.configure(yscrollcommand=planner_scroll.set)

        ttk.Label(self.motion_planner_frame, textvariable=self.motion_plan_status_var).grid(
            row=4, column=0, columnspan=2, sticky="w", pady=(10, 0)
        )

        planner_buttons = ttk.Frame(self.motion_planner_frame)
        planner_buttons.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        for idx in range(4):
            planner_buttons.columnconfigure(idx, weight=1)

        ttk.Button(planner_buttons, text="Run Sequence", command=self.start_motion_plan).grid(
            row=0, column=0, sticky="ew", padx=(0, 6)
        )
        ttk.Button(planner_buttons, text="Stop Sequence", command=self.stop_motion_plan).grid(
            row=0, column=1, sticky="ew", padx=(0, 6)
        )
        ttk.Button(planner_buttons, text="Load Example", command=self.load_sequence_example).grid(
            row=0, column=2, sticky="ew", padx=(0, 6)
        )
        ttk.Button(planner_buttons, text="Insert All Target Step", command=self.copy_all_target_to_planner).grid(
            row=0, column=3, sticky="ew"
        )
        self.motion_planner_frame.grid_remove()
        self.refresh_saved_sequences()

    def _on_container_configure(self, _event: tk.Event) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event: tk.Event) -> None:
        self.canvas.itemconfigure(self.canvas_window, width=event.width)

    def set_status(self, message: str) -> None:
        self.status_var.set(message)

    def refresh_ports(self) -> None:
        ports = list_ports(return_devices_only=True)
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        self.set_status(f"Found {len(ports)} serial port(s).")

    def connect(self) -> None:
        self.disconnect(silent=True)
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("No Port", "Pick a serial port first.")
            return

        try:
            baud = int(self.baud_var.get())
            self.bus = FeetechBus(port=port, baudrate=baud, timeout=0.1)
        except Exception as exc:
            self.bus = None
            messagebox.showerror("Connect Failed", str(exc))
            self.set_status(f"Failed to connect to {port}.")
            return

        self.set_status(f"Connected to {port} at {baud} bps.")

    def disconnect(self, silent: bool = False) -> None:
        self.stop_motion_plan(silent=True)
        if self.bus is not None:
            self.bus.close()
            self.bus = None
        if not silent:
            self.set_status("Disconnected.")

    def require_bus(self) -> FeetechBus | None:
        if self.bus is None:
            messagebox.showwarning("Not Connected", "Connect to the URT-2 first.")
            return None
        return self.bus

    def scan(self) -> None:
        bus = self.require_bus()
        if bus is None:
            return
        self.stop_motion_plan(silent=True)

        try:
            start_id = int(self.scan_start_var.get())
            end_id = int(self.scan_end_var.get())
            found = bus.scan(start_id, end_id)
        except Exception as exc:
            messagebox.showerror("Scan Failed", str(exc))
            self.set_status("Scan failed.")
            return

        self.sync_servo_panels(found)
        self.update_single_id_controls()

        if found:
            self.set_status(f"Found servos: {found}")
            self.refresh_all_feedback()
        else:
            self.set_status("No servos found in scan range.")

    def ensure_servo_panel(self, servo_id: int) -> ServoPanelState:
        existing = self.servo_panels.get(servo_id)
        if existing is not None:
            return existing

        frame = ttk.LabelFrame(self.panels_container, text=finger_label_for_id(servo_id), padding=10)
        row = len(self.servo_panels) + 1
        frame.grid(row=row, column=0, sticky="ew", pady=(0, 10))
        for col in range(8):
            frame.columnconfigure(col, weight=1 if col in (1, 3, 5) else 0)

        advanced_toggle_var = tk.StringVar(value="Show Advanced")
        live_position_var = tk.StringVar(value="Unknown")
        position_var = tk.StringVar(value="180.0")
        jog_step_var = tk.StringVar(value=DEFAULT_JOG_STEP_DEG)
        position_speed_rpm_var = tk.StringVar(value=DEFAULT_POSITION_SPEED_RPM)
        continuous_speed_rpm_var = tk.StringVar(value=DEFAULT_CONTINUOUS_SPEED_RPM)
        accel_deg_s2_var = tk.StringVar(value=DEFAULT_ACCEL_DEG_S2)
        current_limit_a_var = tk.StringVar(value=DEFAULT_CURRENT_LIMIT_A)
        current_var = tk.StringVar(value=DEFAULT_CURRENT_RAW)
        move_speed_guide_var = tk.StringVar()
        accel_guide_var = tk.StringVar()
        current_limit_guide_var = tk.StringVar()
        continuous_speed_guide_var = tk.StringVar()
        feedback_var = tk.StringVar(value="No feedback yet.")

        ttk.Label(frame, text="Live Position (deg)").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=live_position_var).grid(row=0, column=1, sticky="w", padx=(0, 10))

        ttk.Label(frame, text="Target Position (deg)").grid(row=0, column=2, sticky="w")
        position_entry = ttk.Entry(frame, textvariable=position_var, width=10)
        position_entry.grid(row=0, column=3, sticky="ew", padx=(0, 10))
        position_entry.bind("<Return>", lambda _event, sid=servo_id: self.move_servo(sid))
        position_entry.bind("<FocusIn>", lambda _event, entry=position_entry: self.select_entry_text(entry))

        ttk.Label(frame, text="Jog Step (deg)").grid(row=0, column=4, sticky="w")
        ttk.Entry(frame, textvariable=jog_step_var, width=10).grid(row=0, column=5, sticky="ew", padx=(0, 10))

        ttk.Label(frame, text="Move Speed (RPM)").grid(row=0, column=6, sticky="w")
        ttk.Entry(frame, textvariable=position_speed_rpm_var, width=10).grid(row=0, column=7, sticky="ew")

        ttk.Label(frame, textvariable=move_speed_guide_var).grid(
            row=1, column=6, columnspan=2, sticky="w", pady=(4, 0)
        )

        ttk.Label(frame, text="Accel (deg/s^2)").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=accel_deg_s2_var, width=10).grid(
            row=2, column=1, sticky="ew", padx=(0, 10), pady=(8, 0)
        )
        ttk.Label(frame, textvariable=accel_guide_var).grid(
            row=2, column=2, columnspan=2, sticky="w", pady=(8, 0)
        )

        ttk.Label(frame, text="Current Limit (A)").grid(row=2, column=4, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=current_limit_a_var, width=10).grid(
            row=2, column=5, sticky="ew", padx=(0, 10), pady=(8, 0)
        )
        ttk.Label(frame, textvariable=current_limit_guide_var).grid(
            row=2, column=6, columnspan=2, sticky="w", pady=(8, 0)
        )

        pos_buttons = ttk.Frame(frame)
        pos_buttons.grid(row=3, column=0, columnspan=8, sticky="ew", pady=(10, 0))
        for idx in range(8):
            pos_buttons.columnconfigure(idx, weight=1)

        ttk.Button(pos_buttons, text="Refresh", command=lambda sid=servo_id: self.refresh_feedback(sid)).grid(
            row=0, column=0, sticky="ew", padx=(0, 6)
        )
        ttk.Button(pos_buttons, text="Use Live Pos", command=lambda sid=servo_id: self.sync_target_to_live(sid)).grid(
            row=0, column=1, sticky="ew", padx=(0, 6)
        )
        ttk.Button(pos_buttons, text="Home (180 deg)", command=lambda sid=servo_id: self.move_to_degrees(sid, 180.0)).grid(
            row=0, column=2, sticky="ew", padx=(0, 6)
        )
        ttk.Button(pos_buttons, text="Jog -", command=lambda sid=servo_id: self.jog_servo(sid, -1)).grid(
            row=0, column=3, sticky="ew", padx=(0, 6)
        )
        ttk.Button(pos_buttons, text="Jog +", command=lambda sid=servo_id: self.jog_servo(sid, 1)).grid(
            row=0, column=4, sticky="ew", padx=(0, 6)
        )
        ttk.Button(pos_buttons, text="Move To Target", command=lambda sid=servo_id: self.move_servo(sid)).grid(
            row=0, column=5, sticky="ew", padx=(0, 6)
        )
        ttk.Button(pos_buttons, text="Torque On", command=lambda sid=servo_id: self.set_torque(sid, True)).grid(
            row=0, column=6, sticky="ew", padx=(0, 6)
        )
        ttk.Button(pos_buttons, text="Torque Off", command=lambda sid=servo_id: self.set_torque(sid, False)).grid(
            row=0, column=7, sticky="ew"
        )

        helper_row = ttk.Frame(frame)
        helper_row.grid(row=4, column=0, columnspan=8, sticky="ew", pady=(8, 0))
        helper_row.columnconfigure(0, weight=1)
        helper_row.columnconfigure(1, weight=0)
        ttk.Label(helper_row, text="Tip: type a target angle and press Enter to move immediately.").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Button(
            helper_row,
            textvariable=advanced_toggle_var,
            command=lambda sid=servo_id: self.toggle_advanced(sid),
        ).grid(row=0, column=1, sticky="e")

        advanced = ttk.LabelFrame(frame, text="Advanced Continuous Modes", padding=10)
        advanced.grid(row=5, column=0, columnspan=8, sticky="ew", pady=(10, 0))
        for col in range(8):
            advanced.columnconfigure(col, weight=1 if col in (1, 3, 5, 7) else 0)

        ttk.Label(advanced, text="Continuous Speed (RPM)").grid(row=0, column=0, sticky="w")
        ttk.Entry(advanced, textvariable=continuous_speed_rpm_var, width=10).grid(
            row=0, column=1, sticky="ew", padx=(0, 10)
        )
        ttk.Label(advanced, text="Current Raw").grid(row=0, column=2, sticky="w")
        ttk.Entry(advanced, textvariable=current_var, width=10).grid(
            row=0, column=3, sticky="ew", padx=(0, 10)
        )
        ttk.Label(advanced, textvariable=continuous_speed_guide_var).grid(
            row=0, column=4, columnspan=2, sticky="w"
        )
        ttk.Label(
            advanced,
            text="Current raw: start small, about 10-50 for bench testing.",
        ).grid(row=0, column=6, columnspan=2, sticky="w")

        ttk.Button(advanced, text="Run Reverse", command=lambda sid=servo_id: self.speed_servo(sid, -1)).grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0), padx=(0, 6)
        )
        ttk.Button(advanced, text="Stop Speed", command=lambda sid=servo_id: self.stop_speed_servo(sid)).grid(
            row=1, column=2, columnspan=2, sticky="ew", pady=(8, 0), padx=(0, 6)
        )
        ttk.Button(advanced, text="Run Forward", command=lambda sid=servo_id: self.speed_servo(sid, 1)).grid(
            row=1, column=4, columnspan=2, sticky="ew", pady=(8, 0), padx=(0, 6)
        )
        ttk.Button(advanced, text="Send Current", command=lambda sid=servo_id: self.current_servo(sid)).grid(
            row=1, column=6, columnspan=2, sticky="ew", pady=(8, 0)
        )
        advanced.grid_remove()

        ttk.Label(frame, textvariable=feedback_var).grid(
            row=6, column=0, columnspan=8, sticky="ew", pady=(10, 0)
        )

        state = ServoPanelState(
            servo_id=servo_id,
            frame=frame,
            position_entry=position_entry,
            advanced_frame=advanced,
            advanced_toggle_var=advanced_toggle_var,
            live_position_var=live_position_var,
            position_var=position_var,
            jog_step_var=jog_step_var,
            position_speed_rpm_var=position_speed_rpm_var,
            continuous_speed_rpm_var=continuous_speed_rpm_var,
            accel_deg_s2_var=accel_deg_s2_var,
            current_limit_a_var=current_limit_a_var,
            current_var=current_var,
            move_speed_guide_var=move_speed_guide_var,
            accel_guide_var=accel_guide_var,
            current_limit_guide_var=current_limit_guide_var,
            continuous_speed_guide_var=continuous_speed_guide_var,
            feedback_var=feedback_var,
            last_control_mode="servo",
        )
        self.servo_panels[servo_id] = state

        for var in (
            position_speed_rpm_var,
            continuous_speed_rpm_var,
            accel_deg_s2_var,
            current_limit_a_var,
        ):
            var.trace_add("write", lambda *_args, sid=servo_id: self.update_guides(sid))

        self.update_guides(servo_id)
        return state

    def get_state(self, servo_id: int) -> ServoPanelState:
        return self.servo_panels[servo_id]

    def select_entry_text(self, entry: ttk.Entry) -> None:
        entry.focus_set()
        entry.selection_range(0, tk.END)
        entry.icursor(tk.END)

    def toggle_motion_planner(self) -> None:
        if self.motion_planner_frame is None:
            return
        if self.motion_planner_frame.winfo_viewable():
            self.motion_planner_frame.grid_remove()
            self.motion_planner_toggle_var.set("Show Sequence Planner")
        else:
            self.motion_planner_frame.grid()
            self.motion_planner_toggle_var.set("Hide Sequence Planner")

    def copy_all_target_to_planner(self) -> None:
        if self.motion_plan_text is None:
            return
        target = self.clamp_degrees(self.parse_float(self.all_target_deg_var.get(), "All Fingers Target"))
        current_limit = self.parse_float(self.all_current_limit_a_var.get(), "All Fingers Current Limit")
        self.motion_plan_text.insert(
            tk.INSERT,
            f"go to {target:.1f} degrees with a current limit of {current_limit:.2f} A\n",
        )
        self.motion_plan_text.focus_set()

    def load_sequence_example(self) -> None:
        if self.motion_plan_text is None:
            return
        self.motion_plan_text.delete("1.0", tk.END)
        self.motion_plan_text.insert(
            "1.0",
            "# Sequence examples\n"
            "wait 1.5 seconds\n"
            "go to 90 degrees with a current limit of 0.45 A\n"
            "go to 60 degrees with a current limit of 0.70 A\n"
            "wait 5 seconds\n"
            "home\n\n"
            "# Optional finger-specific steps\n"
            "thumb: go to 120 degrees with a current limit of 0.35 A\n"
            "index: home\n"
            "middle: wait 0.5 seconds\n",
        )
        self.motion_plan_text.focus_set()

    def parse_sequence_target(self, target_name: str | None, discovered_servo_ids: list[int]) -> list[int]:
        if target_name is None:
            return discovered_servo_ids

        normalized = target_name.strip().lower()
        if normalized in {"all", "hand", "fingers"}:
            return discovered_servo_ids

        for servo_id in discovered_servo_ids:
            if finger_name_for_id(servo_id).lower() == normalized:
                return [servo_id]

        id_match = re.fullmatch(r"(?:id|finger)\s*(\d+)", normalized)
        if id_match:
            servo_id = int(id_match.group(1))
            if servo_id in discovered_servo_ids:
                return [servo_id]
            raise ValueError(f"Target '{target_name}' is not currently discovered on the bus.")

        raise ValueError(f"Unknown sequence target '{target_name}'. Use all, thumb, index, middle, or id N.")

    def parse_sequence_script(self, script: str, discovered_servo_ids: list[int]) -> list[dict]:
        steps: list[dict] = []
        for line_number, raw_line in enumerate(script.splitlines(), start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            target_name: str | None = None
            line_body = line
            target_match = re.match(r"^(thumb|index|middle|all|hand|fingers|id\s*\d+|finger\s*\d+)\s*:\s*(.+)$", line, re.IGNORECASE)
            if target_match:
                target_name = target_match.group(1)
                line_body = target_match.group(2).strip()

            wait_match = re.fullmatch(r"wait\s+([0-9]*\.?[0-9]+)(?:\s*(?:s|sec|secs|second|seconds))?", line_body, re.IGNORECASE)
            if wait_match:
                steps.append({"type": "wait", "duration_s": float(wait_match.group(1)), "line_number": line_number})
                continue

            home_match = re.fullmatch(r"home", line_body, re.IGNORECASE)
            if home_match:
                steps.append(
                    {
                        "type": "move",
                        "servo_ids": self.parse_sequence_target(target_name, discovered_servo_ids),
                        "target_deg": 180.0,
                        "current_limit_a": None,
                        "line_number": line_number,
                    }
                )
                continue

            move_match = re.fullmatch(
                r"go\s+to\s+(-?[0-9]*\.?[0-9]+)\s*degrees?(?:\s+with\s+a?\s*current\s+limit\s+of\s+([0-9]*\.?[0-9]+)\s*a?)?",
                line_body,
                re.IGNORECASE,
            )
            if move_match:
                current_limit_text = move_match.group(2)
                steps.append(
                    {
                        "type": "move",
                        "servo_ids": self.parse_sequence_target(target_name, discovered_servo_ids),
                        "target_deg": float(move_match.group(1)),
                        "current_limit_a": float(current_limit_text) if current_limit_text is not None else None,
                        "line_number": line_number,
                    }
                )
                continue

            raise ValueError(f"Could not parse line {line_number}: {raw_line}")

        if not steps:
            raise ValueError("Sequence script is empty.")
        return steps

    def eased_alpha(self, alpha: float, easing: str) -> float:
        alpha = max(0.0, min(1.0, alpha))
        if easing == "Linear":
            return alpha
        return alpha * alpha * (3.0 - 2.0 * alpha)

    def sync_servo_panels(self, found_ids: list[int]) -> None:
        found_set = set(found_ids)
        for servo_id in list(self.servo_panels):
            if servo_id not in found_set:
                self.servo_panels[servo_id].frame.destroy()
                del self.servo_panels[servo_id]

        for servo_id in found_ids:
            self.ensure_servo_panel(servo_id)

        for row_index, servo_id in enumerate(sorted(self.servo_panels), start=1):
            self.servo_panels[servo_id].frame.grid_configure(row=row_index)

    def update_single_id_controls(self) -> None:
        servo_ids = self.discovered_servo_ids()
        if len(servo_ids) == 1:
            servo_id = servo_ids[0]
            self.single_id_current_var.set(str(servo_id))
            suggested_id = 2 if servo_id == 1 else servo_id + 1
            if not self.single_id_new_var.get().strip() or self.single_id_new_var.get().strip() == str(servo_id):
                self.single_id_new_var.set(str(min(253, suggested_id)))
            self.single_id_frame.grid()
            return

        self.single_id_current_var.set("-")
        self.single_id_frame.grid_remove()

    def discovered_servo_ids(self) -> list[int]:
        return sorted(self.servo_panels)

    def require_discovered_servos(self) -> list[int] | None:
        servo_ids = self.discovered_servo_ids()
        if not servo_ids:
            messagebox.showwarning("No Servos", "Scan for servos first.")
            return None
        return servo_ids

    def clamp_position(self, value: int) -> int:
        return max(SERVO_MIN_POS, min(SERVO_MAX_POS, value))

    def clamp_degrees(self, value: float) -> float:
        return max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, value))

    def raw_to_degrees(self, raw: int) -> float:
        return raw * RAW_POSITION_TO_DEG

    def degrees_to_raw(self, degrees: float) -> int:
        return self.clamp_position(round(self.clamp_degrees(degrees) / RAW_POSITION_TO_DEG))

    def format_degrees(self, value: float) -> str:
        return f"{value:.1f}"

    def parse_int(self, value: str, label: str) -> int:
        try:
            return int(value)
        except ValueError as exc:
            raise ValueError(f"{label} must be an integer.") from exc

    def parse_float(self, value: str, label: str) -> float:
        try:
            return float(value)
        except ValueError as exc:
            raise ValueError(f"{label} must be a number.") from exc

    def rpm_to_raw(self, rpm: float) -> int:
        return max(0, min(MAX_RAW_SPEED, round(rpm / RAW_SPEED_TO_RPM)))

    def accel_to_raw(self, accel_deg_s2: float) -> int:
        return max(0, min(MAX_RAW_ACCEL, round(accel_deg_s2 / RAW_ACCEL_TO_DEG_S2)))

    def current_limit_a_to_raw(self, current_limit_a: float) -> int:
        return max(
            0,
            min(
                MAX_RAW_CURRENT_LIMIT,
                round((current_limit_a / MAX_MOTOR_CURRENT_A) * MAX_RAW_CURRENT_LIMIT),
            ),
        )

    def update_guides(self, servo_id: int) -> None:
        state = self.get_state(servo_id)

        try:
            move_rpm = self.parse_float(state.position_speed_rpm_var.get(), "Move Speed")
            move_raw = self.rpm_to_raw(move_rpm)
            state.move_speed_guide_var.set(
                f"Suggested 15-45 RPM. Command raw={move_raw} of {MAX_RAW_SPEED} max (~100 RPM @12V)"
            )
        except ValueError:
            state.move_speed_guide_var.set("Suggested 15-45 RPM. Enter a number.")

        try:
            accel_deg_s2 = self.parse_float(state.accel_deg_s2_var.get(), "Accel")
            accel_raw = self.accel_to_raw(accel_deg_s2)
            state.accel_guide_var.set(
                f"Suggested 40-175 deg/s^2. Command raw={accel_raw} of {MAX_RAW_ACCEL} max"
            )
        except ValueError:
            state.accel_guide_var.set("Suggested 40-175 deg/s^2. Enter a number.")

        try:
            current_limit_a = self.parse_float(state.current_limit_a_var.get(), "Current Limit")
            current_limit_raw = self.current_limit_a_to_raw(current_limit_a)
            state.current_limit_guide_var.set(
                f"Suggested 0.3-0.75 A. Command raw={current_limit_raw} of {MAX_RAW_CURRENT_LIMIT} max"
            )
        except ValueError:
            state.current_limit_guide_var.set("Suggested 0.3-0.75 A. Enter a number.")

        try:
            continuous_rpm = self.parse_float(state.continuous_speed_rpm_var.get(), "Continuous Speed")
            continuous_raw = self.rpm_to_raw(abs(continuous_rpm))
            state.continuous_speed_guide_var.set(
                f"Suggested 5-20 RPM for bench testing. Command raw={continuous_raw}"
            )
        except ValueError:
            state.continuous_speed_guide_var.set("Suggested 5-20 RPM. Enter a number.")

    def toggle_advanced(self, servo_id: int) -> None:
        state = self.get_state(servo_id)
        if state.advanced_frame.winfo_viewable():
            state.advanced_frame.grid_remove()
            state.advanced_toggle_var.set("Show Advanced")
        else:
            state.advanced_frame.grid()
            state.advanced_toggle_var.set("Hide Advanced")

    def _run_bus_command(self, action_name: str, func) -> bool:
        bus = self.require_bus()
        if bus is None:
            return False
        try:
            func(bus)
        except Exception as exc:
            messagebox.showerror(f"{action_name} Failed", str(exc))
            self.set_status(f"{action_name} failed.")
            return False
        return True

    def sync_target_to_live(self, servo_id: int) -> None:
        state = self.get_state(servo_id)
        bus = self.require_bus()
        label = finger_label_for_id(servo_id)
        if bus is None:
            return

        try:
            feedback = bus.read_feedback(servo_id)
        except Exception as exc:
            messagebox.showerror("Read Failed", str(exc))
            self.set_status(f"{label} live position read failed.")
            return

        live_deg = self.raw_to_degrees(feedback.position)
        state.position_var.set(self.format_degrees(live_deg))
        state.live_position_var.set(self.format_degrees(live_deg))
        self.refresh_feedback(servo_id, silent=True)
        self.set_status(f"{label} target synced to live position {live_deg:.1f} deg.")

    def move_all_to_degrees(self, target_deg: float) -> None:
        self.all_target_deg_var.set(self.format_degrees(self.clamp_degrees(target_deg)))
        self.move_all_fingers()

    def jog_all_fingers(self, direction: int) -> None:
        bus = self.require_bus()
        servo_ids = self.require_discovered_servos()
        if bus is None or servo_ids is None:
            return

        try:
            step_deg = self.parse_float(self.all_jog_step_deg_var.get(), "All Fingers Jog Step")
            current_target_deg = self.parse_float(self.all_target_deg_var.get(), "All Fingers Target")
        except Exception as exc:
            messagebox.showerror("Jog All Failed", str(exc))
            self.set_status("All fingers jog failed.")
            return

        target_deg = self.clamp_degrees(current_target_deg + direction * step_deg)
        self.all_target_deg_var.set(self.format_degrees(target_deg))
        self.move_all_fingers(status_prefix="All fingers jogged to")

    def move_all_fingers(self, status_prefix: str | None = None) -> None:
        bus = self.require_bus()
        servo_ids = self.require_discovered_servos()
        if bus is None or servo_ids is None:
            return
        self.stop_motion_plan(silent=True)

        try:
            target_deg = self.clamp_degrees(self.parse_float(self.all_target_deg_var.get(), "All Fingers Target"))
            speed_rpm = self.parse_float(self.all_move_speed_rpm_var.get(), "All Fingers Move Speed")
            accel_deg_s2 = self.parse_float(self.all_accel_deg_s2_var.get(), "All Fingers Accel")
            current_limit_a = self.parse_float(self.all_current_limit_a_var.get(), "All Fingers Current Limit")
        except Exception as exc:
            messagebox.showerror("Move All Failed", str(exc))
            self.set_status("All fingers move failed.")
            return

        use_soft_home = target_deg == 180.0 and any(
            self.get_state(servo_id).last_control_mode in {"speed", "current"} for servo_id in servo_ids
        )
        effective_speed_rpm = SOFT_HOME_SPEED_RPM if use_soft_home else speed_rpm
        effective_accel_deg_s2 = SOFT_HOME_ACCEL_DEG_S2 if use_soft_home else accel_deg_s2
        effective_current_limit_a = SOFT_HOME_CURRENT_LIMIT_A if use_soft_home else current_limit_a

        speed_raw = self.rpm_to_raw(effective_speed_rpm)
        accel_raw = self.accel_to_raw(effective_accel_deg_s2)
        current_limit_raw = self.current_limit_a_to_raw(effective_current_limit_a)
        target_raw = self.degrees_to_raw(target_deg)

        def command(active_bus: FeetechBus) -> None:
            for servo_id in servo_ids:
                active_bus.set_mode(servo_id, HLSMode.SERVO)
                active_bus.enable_torque(servo_id, True)
                active_bus.set_position(
                    servo_id=servo_id,
                    position=target_raw,
                    speed=speed_raw,
                    acc=accel_raw,
                    torque_limit_raw=current_limit_raw,
                    reg_write=True,
                )
            active_bus.action()

        if not self._run_bus_command("Move All", command):
            return

        for servo_id in servo_ids:
            state = self.get_state(servo_id)
            state.position_var.set(self.format_degrees(target_deg))
            state.last_control_mode = "servo"

        self.all_target_deg_var.set(self.format_degrees(target_deg))
        if status_prefix is not None:
            prefix = status_prefix
        elif use_soft_home:
            prefix = "All fingers soft homing to"
        else:
            prefix = "All fingers moving to"
        self.set_status(f"{prefix} {target_deg:.1f} deg at speed {effective_speed_rpm:.1f} RPM.")
        self.select_entry_text(self.all_target_entry)
        self.refresh_all_feedback()

    def set_torque_all(self, enabled: bool) -> None:
        servo_ids = self.require_discovered_servos()
        if servo_ids is None:
            return
        self.stop_motion_plan(silent=True)

        def command(bus: FeetechBus) -> None:
            for servo_id in servo_ids:
                bus.enable_torque(servo_id, enabled)

        if not self._run_bus_command("Torque All", command):
            return
        self.set_status(f"All discovered servos torque {'enabled' if enabled else 'disabled'}.")
        self.refresh_all_feedback()

    def start_motion_plan(
        self,
        target_override: float | None = None,
        use_bend_limit_as_target: bool = False,
    ) -> None:
        bus = self.require_bus()
        servo_ids = self.require_discovered_servos()
        if bus is None or servo_ids is None:
            return

        self.stop_motion_plan(silent=True)

        try:
            bend_limit_deg = self.clamp_degrees(
                self.parse_float(self.motion_plan_bend_limit_deg_var.get(), "Most Closed")
            )
            script = self.motion_plan_text.get("1.0", tk.END) if self.motion_plan_text is not None else ""
            if target_override is not None:
                script = f"go to {float(target_override):.1f} degrees\n"
            elif use_bend_limit_as_target:
                script = f"go to {bend_limit_deg:.1f} degrees\n"
            parsed_steps = self.parse_sequence_script(script, servo_ids)
            current_positions = {
                servo_id: self.raw_to_degrees(bus.read_feedback(servo_id).position) for servo_id in servo_ids
            }
            default_speed_rpm = self.parse_float(self.all_move_speed_rpm_var.get(), "Move Speed")
            default_accel_deg_s2 = self.parse_float(self.all_accel_deg_s2_var.get(), "Accel")
            default_current_limit_a = self.parse_float(self.all_current_limit_a_var.get(), "Current Limit")
        except Exception as exc:
            messagebox.showerror("Sequence Failed", str(exc))
            self.motion_plan_status_var.set(str(exc))
            return

        self.motion_plan_state = {
            "servo_ids": servo_ids,
            "parsed_steps": parsed_steps,
            "step_index": 0,
            "bend_limit_deg": bend_limit_deg,
            "current_positions": current_positions,
            "default_speed_rpm": default_speed_rpm,
            "default_accel_deg_s2": default_accel_deg_s2,
            "default_current_limit_a": default_current_limit_a,
        }
        self.motion_plan_status_var.set(
            f"Running sequence with {len(parsed_steps)} step(s). Bend limit = {bend_limit_deg:.1f} deg."
        )
        self._run_motion_plan_step()

    def _run_motion_plan_step(self) -> None:
        if self.motion_plan_state is None:
            return

        state = self.motion_plan_state
        parsed_steps: list[dict] = state["parsed_steps"]
        if state["step_index"] >= len(parsed_steps):
            self.motion_plan_status_var.set("Sequence complete.")
            self.motion_plan_state = None
            self.motion_plan_after_id = None
            if self.motion_plan_text is not None:
                self.motion_plan_text.focus_set()
            self.refresh_all_feedback()
            return

        step = parsed_steps[state["step_index"]]
        state["step_index"] += 1

        if step["type"] == "wait":
            duration_ms = max(0, int(step["duration_s"] * 1000.0))
            self.motion_plan_status_var.set(
                f"Step {state['step_index']}/{len(parsed_steps)}: waiting {step['duration_s']:.2f} s."
            )
            self.motion_plan_after_id = self.root.after(duration_ms, self._run_motion_plan_step)
            return

        servo_ids: list[int] = step["servo_ids"]
        target_deg = max(state["bend_limit_deg"], self.clamp_degrees(step["target_deg"]))
        current_limit_a = (
            step["current_limit_a"]
            if step["current_limit_a"] is not None
            else state["default_current_limit_a"]
        )
        use_soft_home = target_deg == 180.0 and any(
            self.get_state(servo_id).last_control_mode in {"speed", "current"} for servo_id in servo_ids
        )
        effective_speed_rpm = SOFT_HOME_SPEED_RPM if use_soft_home else state["default_speed_rpm"]
        effective_accel_deg_s2 = SOFT_HOME_ACCEL_DEG_S2 if use_soft_home else state["default_accel_deg_s2"]
        effective_current_limit_a = SOFT_HOME_CURRENT_LIMIT_A if use_soft_home else current_limit_a

        speed_raw = self.rpm_to_raw(effective_speed_rpm)
        accel_raw = self.accel_to_raw(effective_accel_deg_s2)
        current_limit_raw = self.current_limit_a_to_raw(effective_current_limit_a)
        target_raw = self.degrees_to_raw(target_deg)

        def command(bus: FeetechBus) -> None:
            for servo_id in servo_ids:
                bus.set_mode(servo_id, HLSMode.SERVO)
                bus.enable_torque(servo_id, True)
                bus.set_position(
                    servo_id=servo_id,
                    position=target_raw,
                    speed=speed_raw,
                    acc=accel_raw,
                    torque_limit_raw=current_limit_raw,
                    reg_write=len(servo_ids) > 1,
                )
            if len(servo_ids) > 1:
                bus.action()

        if not self._run_bus_command("Sequence Step", command):
            self.stop_motion_plan()
            return

        step_label = (
            "all fingers"
            if len(servo_ids) == len(state["servo_ids"])
            else ", ".join(finger_name_for_id(servo_id) for servo_id in servo_ids)
        )
        self.motion_plan_status_var.set(
            f"Step {state['step_index']}/{len(parsed_steps)}: moving {step_label} to {target_deg:.1f} deg with {effective_current_limit_a:.2f} A."
        )

        max_distance_deg = max(abs(target_deg - state["current_positions"][servo_id]) for servo_id in servo_ids)
        for servo_id in servo_ids:
            state["current_positions"][servo_id] = target_deg
            panel_state = self.get_state(servo_id)
            panel_state.position_var.set(self.format_degrees(target_deg))
            panel_state.last_control_mode = "servo"
        if len(servo_ids) == len(state["servo_ids"]):
            self.all_target_deg_var.set(self.format_degrees(target_deg))

        estimated_motion_s = max(0.2, (max_distance_deg / max(1.0, effective_speed_rpm * 6.0)) + 0.2)
        self.motion_plan_after_id = self.root.after(int(estimated_motion_s * 1000.0), self._run_motion_plan_step)

    def stop_motion_plan(self, silent: bool = False) -> None:
        if self.motion_plan_after_id is not None:
            self.root.after_cancel(self.motion_plan_after_id)
            self.motion_plan_after_id = None
        was_running = self.motion_plan_state is not None
        self.motion_plan_state = None
        if was_running and not silent:
            self.motion_plan_status_var.set("Sequence stopped.")

    def set_single_servo_id(self) -> None:
        bus = self.require_bus()
        servo_ids = self.require_discovered_servos()
        if bus is None or servo_ids is None:
            return
        if len(servo_ids) != 1:
            messagebox.showwarning("Single Servo Only", "Connect or scan exactly one servo before changing its ID.")
            return

        current_id = servo_ids[0]
        try:
            new_id = self.parse_int(self.single_id_new_var.get(), "New ID")
        except Exception as exc:
            messagebox.showerror("Set ID Failed", str(exc))
            return

        if not 1 <= new_id <= 253:
            messagebox.showwarning("Invalid ID", "Choose a new servo ID between 1 and 253.")
            return
        if new_id == current_id:
            self.set_status(f"{finger_label_for_id(current_id)} already has ID {new_id}.")
            return

        def command(active_bus: FeetechBus) -> None:
            active_bus.set_id(current_id, new_id)

        if not self._run_bus_command("Set ID", command):
            return

        self.set_status(
            f"{finger_label_for_id(current_id)} ID changed to {new_id}. Scan again to rediscover it on the bus."
        )
        self.single_id_current_var.set(str(new_id))
        self.single_id_new_var.set(str(min(253, new_id + 1)))
        self.sync_servo_panels([])
        self.update_single_id_controls()

    def move_to_degrees(self, servo_id: int, target_deg: float) -> None:
        state = self.get_state(servo_id)
        target_deg = self.clamp_degrees(target_deg)
        state.position_var.set(self.format_degrees(target_deg))
        if target_deg == 180.0 and state.last_control_mode in {"speed", "current"}:
            self.move_servo(
                servo_id,
                status_prefix=f"{finger_label_for_id(servo_id)} soft homing to",
                speed_rpm_override=SOFT_HOME_SPEED_RPM,
                accel_deg_s2_override=SOFT_HOME_ACCEL_DEG_S2,
                current_limit_a_override=SOFT_HOME_CURRENT_LIMIT_A,
            )
            return
        self.move_servo(servo_id)

    def jog_servo(self, servo_id: int, direction: int) -> None:
        state = self.get_state(servo_id)
        bus = self.require_bus()
        label = finger_label_for_id(servo_id)
        if bus is None:
            return

        try:
            feedback = bus.read_feedback(servo_id)
            step_deg = self.parse_float(state.jog_step_var.get(), "Jog Step")
        except Exception as exc:
            messagebox.showerror("Jog Failed", str(exc))
            self.set_status(f"{label} jog failed.")
            return

        live_deg = self.raw_to_degrees(feedback.position)
        target_deg = self.clamp_degrees(live_deg + direction * step_deg)
        state.position_var.set(self.format_degrees(target_deg))
        state.live_position_var.set(self.format_degrees(live_deg))
        self.move_servo(servo_id, status_prefix=f"{label} jogged to")

    def move_servo(
        self,
        servo_id: int,
        status_prefix: str | None = None,
        speed_rpm_override: float | None = None,
        accel_deg_s2_override: float | None = None,
        current_limit_a_override: float | None = None,
    ) -> None:
        state = self.get_state(servo_id)
        label = finger_label_for_id(servo_id)
        self.stop_motion_plan(silent=True)

        def command(bus: FeetechBus) -> None:
            speed_rpm = (
                speed_rpm_override
                if speed_rpm_override is not None
                else self.parse_float(state.position_speed_rpm_var.get(), "Move Speed")
            )
            accel_deg_s2 = (
                accel_deg_s2_override
                if accel_deg_s2_override is not None
                else self.parse_float(state.accel_deg_s2_var.get(), "Accel")
            )
            current_limit_a = (
                current_limit_a_override
                if current_limit_a_override is not None
                else self.parse_float(state.current_limit_a_var.get(), "Current Limit")
            )
            bus.set_mode(servo_id, HLSMode.SERVO)
            bus.enable_torque(servo_id, True)
            bus.set_position(
                servo_id=servo_id,
                position=self.degrees_to_raw(self.parse_float(state.position_var.get(), "Target Position")),
                speed=self.rpm_to_raw(speed_rpm),
                acc=self.accel_to_raw(accel_deg_s2),
                torque_limit_raw=self.current_limit_a_to_raw(current_limit_a),
            )

        if not self._run_bus_command("Move", command):
            return

        target_deg = self.clamp_degrees(self.parse_float(state.position_var.get(), "Target Position"))
        state.position_var.set(self.format_degrees(target_deg))
        state.last_control_mode = "servo"
        prefix = status_prefix or f"{label} moving to"
        status_speed_rpm = (
            speed_rpm_override
            if speed_rpm_override is not None
            else self.parse_float(state.position_speed_rpm_var.get(), "Move Speed")
        )
        self.set_status(f"{prefix} {state.position_var.get()} deg at speed {status_speed_rpm:.1f} RPM.")
        self.select_entry_text(state.position_entry)
        self.refresh_feedback(servo_id, silent=True)

    def speed_servo(self, servo_id: int, direction: int) -> None:
        state = self.get_state(servo_id)
        label = finger_label_for_id(servo_id)
        self.stop_motion_plan(silent=True)

        def command(bus: FeetechBus) -> None:
            bus.set_mode(servo_id, HLSMode.SPEED)
            bus.enable_torque(servo_id, True)
            bus.set_speed(
                servo_id=servo_id,
                speed=direction * self.rpm_to_raw(self.parse_float(state.continuous_speed_rpm_var.get(), "Continuous Speed")),
                acc=self.accel_to_raw(self.parse_float(state.accel_deg_s2_var.get(), "Accel")),
                torque_limit_raw=self.current_limit_a_to_raw(
                    self.parse_float(state.current_limit_a_var.get(), "Current Limit")
                ),
            )

        if not self._run_bus_command("Speed", command):
            return
        state.last_control_mode = "speed"
        direction_name = "forward" if direction > 0 else "reverse"
        self.set_status(
            f"{label} running {direction_name} in continuous speed mode at {state.continuous_speed_rpm_var.get()} RPM."
        )
        self.refresh_feedback(servo_id, silent=True)

    def stop_speed_servo(self, servo_id: int) -> None:
        state = self.get_state(servo_id)
        label = finger_label_for_id(servo_id)
        self.stop_motion_plan(silent=True)

        def command(bus: FeetechBus) -> None:
            bus.set_mode(servo_id, HLSMode.SPEED)
            bus.enable_torque(servo_id, True)
            bus.set_speed(
                servo_id=servo_id,
                speed=0,
                acc=self.accel_to_raw(self.parse_float(state.accel_deg_s2_var.get(), "Accel")),
                torque_limit_raw=self.current_limit_a_to_raw(
                    self.parse_float(state.current_limit_a_var.get(), "Current Limit")
                ),
            )

        if not self._run_bus_command("Stop Speed", command):
            return
        state.last_control_mode = "speed"
        self.set_status(f"{label} continuous speed command stopped.")
        self.refresh_feedback(servo_id, silent=True)

    def current_servo(self, servo_id: int) -> None:
        state = self.get_state(servo_id)
        label = finger_label_for_id(servo_id)
        self.stop_motion_plan(silent=True)

        def command(bus: FeetechBus) -> None:
            bus.set_mode(servo_id, HLSMode.CURRENT)
            bus.enable_torque(servo_id, True)
            bus.set_current_raw(servo_id, self.parse_int(state.current_var.get(), "Current Raw"))

        if not self._run_bus_command("Current", command):
            return
        state.last_control_mode = "current"
        self.set_status(f"{label} current command set to {state.current_var.get()} raw units.")
        self.refresh_feedback(servo_id, silent=True)

    def set_torque(self, servo_id: int, enabled: bool) -> None:
        label = finger_label_for_id(servo_id)
        self.stop_motion_plan(silent=True)

        def command(bus: FeetechBus) -> None:
            bus.enable_torque(servo_id, enabled)

        if not self._run_bus_command("Torque", command):
            return
        self.set_status(f"{label} torque {'enabled' if enabled else 'disabled'}.")
        self.refresh_feedback(servo_id, silent=True)

    def refresh_feedback(self, servo_id: int, silent: bool = False) -> None:
        state = self.get_state(servo_id)
        bus = self.require_bus()
        label = finger_label_for_id(servo_id)
        if bus is None:
            return

        try:
            feedback = bus.read_feedback(servo_id)
        except Exception as exc:
            state.feedback_var.set(f"Read failed: {exc}")
            if not silent:
                self.set_status(f"{label} refresh failed.")
            return

        state.feedback_var.set(
            "pos={:.1f} deg ({} raw) speed={} load={} voltage={:.1f}V temp={}C moving={} current_raw={}".format(
                self.raw_to_degrees(feedback.position),
                feedback.position,
                feedback.speed,
                feedback.load,
                feedback.voltage_v,
                feedback.temperature_c,
                feedback.moving,
                feedback.current_raw,
            )
        )
        state.live_position_var.set(self.format_degrees(self.raw_to_degrees(feedback.position)))
        if not silent:
            self.set_status(f"{label} feedback refreshed.")

    def refresh_all_feedback(self) -> None:
        for servo_id in list(self.servo_panels):
            self.refresh_feedback(servo_id, silent=True)

    def _schedule_refresh(self) -> None:
        self.root.after(self._refresh_interval_ms(), self._auto_refresh_tick)

    def _refresh_interval_ms(self) -> int:
        try:
            value = int(self.refresh_ms_var.get())
        except ValueError:
            value = 500
        return max(100, value)

    def _auto_refresh_tick(self) -> None:
        if self.auto_refresh_var.get() and self.bus is not None and self.servo_panels and self.motion_plan_state is None:
            self.refresh_all_feedback()
        self._schedule_refresh()


def build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Standalone Feetech HLS controller.")
    parser.add_argument("--port", help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Bus baud rate")
    parser.add_argument("--timeout", type=float, default=0.1, help="Serial timeout in seconds")

    sub = parser.add_subparsers(dest="command")
    sub.add_parser("gui", help="Launch the desktop controller")
    sub.add_parser("ports", help="List available serial ports")

    p_scan = sub.add_parser("scan", help="Ping a range of servo IDs")
    p_scan.add_argument("--start", type=int, default=0)
    p_scan.add_argument("--end", type=int, default=20)

    p_ping = sub.add_parser("ping", help="Ping one servo")
    p_ping.add_argument("--id", type=int, required=True)

    p_read = sub.add_parser("read", help="Read one servo's feedback block")
    p_read.add_argument("--id", type=int, required=True)

    p_move = sub.add_parser("move", help="Position move in degrees")
    p_move.add_argument("--id", type=int, required=True)
    p_move.add_argument("--degrees", type=float, required=True)
    p_move.add_argument("--speed-rpm", type=float, default=float(DEFAULT_POSITION_SPEED_RPM))
    p_move.add_argument("--accel-deg-s2", type=float, default=float(DEFAULT_ACCEL_DEG_S2))
    p_move.add_argument("--current-limit-a", type=float, default=float(DEFAULT_CURRENT_LIMIT_A))

    p_speed = sub.add_parser("speed", help="Continuous speed mode")
    p_speed.add_argument("--id", type=int, required=True)
    p_speed.add_argument("--rpm", type=float, required=True)
    p_speed.add_argument("--accel-deg-s2", type=float, default=float(DEFAULT_ACCEL_DEG_S2))
    p_speed.add_argument("--current-limit-a", type=float, default=float(DEFAULT_CURRENT_LIMIT_A))

    p_current = sub.add_parser("current", help="Current mode raw command")
    p_current.add_argument("--id", type=int, required=True)
    p_current.add_argument("--raw", type=int, required=True)

    p_torque = sub.add_parser("torque", help="Enable or disable the servo output stage")
    p_torque.add_argument("--id", type=int, required=True)
    p_torque.add_argument("--enable", action="store_true")
    p_torque.add_argument("--disable", action="store_true")

    p_monitor = sub.add_parser("monitor", help="Continuously print feedback")
    p_monitor.add_argument("--id", type=int, required=True)
    p_monitor.add_argument("--interval", type=float, default=0.2)

    return parser


def require_port(args: argparse.Namespace) -> str:
    if args.port:
        return args.port
    raise SystemExit("--port is required for this command.")


def raw_to_degrees(raw: int) -> float:
    return raw * RAW_POSITION_TO_DEG


def rpm_to_raw(rpm: float) -> int:
    return max(0, min(MAX_RAW_SPEED, round(rpm / RAW_SPEED_TO_RPM)))


def accel_to_raw(accel_deg_s2: float) -> int:
    return max(0, min(MAX_RAW_ACCEL, round(accel_deg_s2 / RAW_ACCEL_TO_DEG_S2)))


def current_limit_a_to_raw(current_limit_a: float) -> int:
    return max(
        0,
        min(
            MAX_RAW_CURRENT_LIMIT,
            round((current_limit_a / MAX_MOTOR_CURRENT_A) * MAX_RAW_CURRENT_LIMIT),
        ),
    )


def degrees_to_raw(degrees: float) -> int:
    clamped = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, degrees))
    return max(SERVO_MIN_POS, min(SERVO_MAX_POS, round(clamped / RAW_POSITION_TO_DEG)))


def print_feedback(feedback: Feedback) -> None:
    print(
        "pos={:.1f} deg ({} raw) speed={} load={} voltage={:.1f}V temp={}C moving={} current_raw={}".format(
            raw_to_degrees(feedback.position),
            feedback.position,
            feedback.speed,
            feedback.load,
            feedback.voltage_v,
            feedback.temperature_c,
            feedback.moving,
            feedback.current_raw,
        )
    )


def launch_gui() -> int:
    root = tk.Tk()
    style = ttk.Style(root)
    if "vista" in style.theme_names():
        style.theme_use("vista")
    app = ServoControlApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(silent=True), root.destroy()))
    root.mainloop()
    return 0


def run_cli(args: argparse.Namespace) -> int:
    if args.command in (None, "gui"):
        return launch_gui()

    if args.command == "ports":
        list_ports()
        return 0

    port = require_port(args)
    with FeetechBus(port=port, baudrate=args.baud, timeout=args.timeout) as bus:
        if args.command == "scan":
            found = bus.scan(args.start, args.end)
            print("Found servos:" if found else "No servos found.", found)
            return 0

        if args.command == "ping":
            bus.ping(args.id)
            print(f"{finger_label_for_id(args.id)} responded.")
            return 0

        if args.command == "read":
            print_feedback(bus.read_feedback(args.id))
            return 0

        if args.command == "move":
            bus.set_mode(args.id, HLSMode.SERVO)
            bus.enable_torque(args.id, True)
            bus.set_position(
                servo_id=args.id,
                position=degrees_to_raw(args.degrees),
                speed=rpm_to_raw(args.speed_rpm),
                acc=accel_to_raw(args.accel_deg_s2),
                torque_limit_raw=current_limit_a_to_raw(args.current_limit_a),
            )
            print(f"{finger_label_for_id(args.id)} moving to {args.degrees:.1f} deg.")
            return 0

        if args.command == "speed":
            bus.set_mode(args.id, HLSMode.SPEED)
            bus.enable_torque(args.id, True)
            direction = -1 if args.rpm < 0 else 1
            bus.set_speed(
                servo_id=args.id,
                speed=direction * rpm_to_raw(abs(args.rpm)),
                acc=accel_to_raw(args.accel_deg_s2),
                torque_limit_raw=current_limit_a_to_raw(args.current_limit_a),
            )
            print(f"{finger_label_for_id(args.id)} continuous speed set to {args.rpm:.1f} RPM.")
            return 0

        if args.command == "current":
            bus.set_mode(args.id, HLSMode.CURRENT)
            bus.enable_torque(args.id, True)
            bus.set_current_raw(args.id, args.raw)
            print(f"{finger_label_for_id(args.id)} current raw set to {args.raw}.")
            return 0

        if args.command == "torque":
            if args.enable == args.disable:
                raise SystemExit("Choose exactly one of --enable or --disable.")
            bus.enable_torque(args.id, args.enable)
            print(f"{finger_label_for_id(args.id)} torque {'enabled' if args.enable else 'disabled'}.")
            return 0

        if args.command == "monitor":
            try:
                while True:
                    print_feedback(bus.read_feedback(args.id))
                    time.sleep(args.interval)
            except KeyboardInterrupt:
                return 0

    raise SystemExit(f"Unsupported command: {args.command}")


def main() -> int:
    parser = build_cli_parser()
    args = parser.parse_args()
    return run_cli(args)


if __name__ == "__main__":
    sys.exit(main())
