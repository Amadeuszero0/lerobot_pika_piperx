#!/usr/bin/env python3
"""Read-only staged diagnostics for the site's D435i, Pika, and Piper hardware.

The default scan only inspects Linux device nodes and network interfaces.  Pass
``--live`` to open the configured Pika serial ports and Piper CAN interfaces and
read feedback.  The live probe never enables an arm and never sends a motion
command.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import yaml


SIDES = ("left", "right")
D435I_USB_ID = "8086:0b3a"
DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "piper"
    / "dual_pika_piper_local.yaml"
)


@dataclass
class Check:
    level: str
    name: str
    detail: str


class Report:
    def __init__(self) -> None:
        self.checks: list[Check] = []

    def add(self, level: str, name: str, detail: str) -> None:
        self.checks.append(Check(level, name, detail))
        marker = {"ok": "OK", "warn": "WARN", "fail": "FAIL", "info": "INFO"}[level]
        print(f"[{marker:4}] {name}: {detail}")

    @property
    def failed(self) -> bool:
        return any(check.level == "fail" for check in self.checks)

    def count(self, level: str) -> int:
        return sum(check.level == level for check in self.checks)


def run_command(command: list[str], timeout_s: float = 8.0) -> tuple[int, str]:
    executable = shutil.which(command[0])
    if executable is None:
        return 127, f"command not found: {command[0]}"
    try:
        completed = subprocess.run(
            [executable, *command[1:]],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as exc:
        partial = (exc.stdout or "") + (exc.stderr or "")
        return 124, f"timed out after {timeout_s:.1f}s\n{partial}".strip()
    output = "\n".join(part for part in (completed.stdout, completed.stderr) if part).strip()
    return completed.returncode, output


def load_config(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    robots = data.get("robot", {}).get("robots", {})
    teleops = data.get("teleop", {}).get("teleops", {})
    if set(robots) != set(SIDES) or set(teleops) != set(SIDES):
        raise ValueError("configuration must contain left/right robots and teleops")
    return data


def resolve_device(path_string: str) -> tuple[bool, str]:
    path = Path(path_string)
    if not path.exists():
        if path.is_symlink():
            return False, f"dangling symlink -> {os.readlink(path)}"
        return False, "missing"
    try:
        return True, str(path.resolve(strict=True))
    except OSError as exc:
        return False, str(exc)


def scan_expected_paths(config: dict[str, Any], report: Report) -> None:
    print("\n== Expected Pika serial paths ==")
    for side in SIDES:
        path = str(config["teleop"]["teleops"][side]["port"])
        exists, resolved = resolve_device(path)
        report.add("ok" if exists else "fail", f"Pika {side}", f"{path} -> {resolved}")

    serial_dir = Path("/dev/serial/by-id")
    if serial_dir.is_dir():
        entries = []
        for entry in sorted(serial_dir.iterdir()):
            try:
                entries.append(f"{entry} -> {entry.resolve()}")
            except OSError:
                entries.append(f"{entry} -> BROKEN")
        report.add("info", "All serial devices", "\n    " + "\n    ".join(entries) if entries else "none")
    else:
        report.add("warn", "All serial devices", "/dev/serial/by-id does not exist")


def scan_can(config: dict[str, Any], report: Report) -> None:
    print("\n== Piper CAN interfaces ==")
    code, all_can = run_command(["ip", "-details", "-statistics", "link", "show", "type", "can"])
    if code == 0:
        report.add("info", "All CAN interfaces", "\n    " + all_can.replace("\n", "\n    ") if all_can else "none")
    else:
        report.add("warn", "All CAN interfaces", all_can)
    for side in SIDES:
        interface = str(config["robot"]["robots"][side]["port"])
        sys_path = Path("/sys/class/net") / interface
        if not sys_path.exists():
            report.add("fail", f"Piper {side}", f"CAN interface {interface!r} is missing")
            continue
        code, output = run_command(["ip", "-details", "-statistics", "link", "show", "dev", interface])
        if code != 0:
            report.add("fail", f"Piper {side}", f"cannot inspect {interface}: {output}")
            continue
        is_up = "state UP" in output or "<UP," in output or ",UP>" in output
        bitrate_ok = "bitrate 1000000" in output
        level = "ok" if is_up and bitrate_ok else "fail"
        detail = f"{interface} is {'UP' if is_up else 'DOWN'}, "
        detail += "1 Mbps" if bitrate_ok else "bitrate is not 1 Mbps or could not be read"
        report.add(level, f"Piper {side}", detail)
        print("    " + output.replace("\n", "\n    "))


def _realsense_from_pyrealsense() -> tuple[list[dict[str, str]], str | None]:
    try:
        import pyrealsense2 as rs
    except Exception as exc:
        return [], f"pyrealsense2 unavailable: {type(exc).__name__}: {exc}"

    fields: list[tuple[str, Any]] = [
        ("name", rs.camera_info.name),
        ("serial", rs.camera_info.serial_number),
        ("firmware", rs.camera_info.firmware_version),
        ("usb", rs.camera_info.usb_type_descriptor),
        ("physical_port", rs.camera_info.physical_port),
    ]
    devices: list[dict[str, str]] = []
    try:
        for device in rs.context().query_devices():
            info: dict[str, str] = {}
            for key, field in fields:
                try:
                    if device.supports(field):
                        info[key] = device.get_info(field)
                except Exception:
                    pass
            # This staged load test is specifically for D435i cameras.  Other
            # connected RealSense models (for example the site's D405 pair)
            # must not affect --expected-realsense.
            if "D435I" in info.get("name", "").upper():
                devices.append(info)
    except Exception as exc:
        return [], f"RealSense enumeration failed: {type(exc).__name__}: {exc}"
    return devices, None


def _realsense_from_usb() -> tuple[int | None, str]:
    # 8086:0b3a is the D435i product ID.  Counting every Intel USB device also
    # counts unrelated RealSense models (this host has two D405 cameras).
    code, output = run_command(["lsusb", "-d", D435I_USB_ID])
    if code == 127:
        return None, output
    if code not in (0, 1):
        return None, output
    lines = [line for line in output.splitlines() if line.strip()]
    return len(lines), output or "none"


def scan_realsense(expected_count: int, report: Report) -> None:
    print("\n== Intel RealSense cameras ==")
    devices, error = _realsense_from_pyrealsense()
    if devices:
        report.add(
            "ok" if len(devices) == expected_count else "fail",
            "RealSense SDK",
            f"found {len(devices)}/{expected_count}: {json.dumps(devices, ensure_ascii=False)}",
        )
        slow_devices = [
            device
            for device in devices
            if device.get("name", "").upper().endswith("D435I")
            and not device.get("usb", "").startswith("3")
        ]
        if slow_devices:
            serials = ", ".join(device.get("serial", "unknown") for device in slow_devices)
            report.add(
                "warn",
                "D435i USB speed",
                f"USB 2.x negotiated by: {serials}; validate all three 640x480@30 streams together",
            )
    else:
        level = "warn" if error and "unavailable" in error else "fail"
        report.add(level, "RealSense SDK", error or f"found 0/{expected_count}")

    usb_count, usb_output = _realsense_from_usb()
    if usb_count is None:
        report.add("warn", "RealSense USB", usb_output)
    else:
        report.add(
            "ok" if usb_count == expected_count else "fail",
            "RealSense USB",
            f"found {usb_count}/{expected_count}\n    " + usb_output.replace("\n", "\n    "),
        )

    code, tree = run_command(["lsusb", "-t"])
    if code == 0:
        report.add("info", "USB topology", "\n    " + tree.replace("\n", "\n    "))
        if "5000M" not in tree and "10000M" not in tree and usb_count:
            report.add("warn", "USB speed", "no SuperSpeed (5000M/10000M) branch is visible")
    else:
        report.add("warn", "USB topology", tree)

    code, all_usb = run_command(["lsusb"])
    if code == 0:
        report.add("info", "All USB devices", "\n    " + all_usb.replace("\n", "\n    "))
    else:
        report.add("warn", "All USB devices", all_usb)


def _wait_until(function: Callable[[], Any], timeout_s: float) -> Any:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return function()
        except Exception as exc:
            last_error = exc
            time.sleep(0.05)
    if last_error is not None:
        raise last_error
    raise TimeoutError(f"no feedback within {timeout_s:.1f}s")


def probe_pipers(config: dict[str, Any], report: Report) -> None:
    print("\n== Live Piper feedback (no enable, no motion command) ==")
    try:
        from lerobot_real.devices.piper.piper_motors_bus import PiperMotorsBus
    except Exception as exc:
        report.add("fail", "Piper SDK", f"import failed: {type(exc).__name__}: {exc}")
        return

    for side in SIDES:
        port = str(config["robot"]["robots"][side]["port"])
        bus = None
        try:
            bus = PiperMotorsBus(
                id=f"{side}_diagnostic",
                port=port,
                motors={},
                calibration={},
                feedback_timeout_s=1.0,
            )
            bus.connect(handshake=False)

            def read_feedback() -> dict[str, Any]:
                status = bus.get_arm_status()
                motors = bus.get_motor_status(require_enabled=False)
                pose = bus.get_end_pose()
                enabled = [
                    bool(getattr(motors, f"motor_{index}").foc_status.driver_enable_status)
                    for index in range(1, 7)
                ]
                return {
                    "arm_status": int(status.arm_status),
                    "error_code": int(status.err_code),
                    "motors_enabled": enabled,
                    "end_pose_xyz_mm_rpy_deg": [round(float(value), 3) for value in pose],
                }

            result = _wait_until(read_feedback, 5.0)
            report.add("ok", f"Piper {side} live", json.dumps(result, ensure_ascii=False))
        except Exception as exc:
            report.add("fail", f"Piper {side} live", f"{type(exc).__name__}: {exc}")
        finally:
            if bus is not None and getattr(bus, "is_connected", False):
                try:
                    # DisconnectPort closes the passive SDK connection.  Do not call
                    # bus.disconnect(), because its options include torque handling.
                    bus.piper.DisconnectPort()
                except Exception as exc:
                    report.add("warn", f"Piper {side} close", str(exc))


def probe_pikas(config: dict[str, Any], report: Report) -> None:
    print("\n== Live Pika feedback (read only) ==")
    try:
        from lerobot_real.devices.pika import PikaDevice
    except Exception as exc:
        report.add("fail", "Pika SDK", f"import failed: {type(exc).__name__}: {exc}")
        return

    devices: dict[str, Any] = {}
    for side in SIDES:
        teleop = config["teleop"]["teleops"][side]
        try:
            device = PikaDevice(
                1,
                pika_sense_port=str(teleop["port"]),
                pika_tracker_device=str(teleop["tracker_device_id"]),
            )
            devices[side] = device
            sense = device.pika_sense
            pose = sense.get_pose(device.pika_tracker_device)
            if pose is None:
                raise RuntimeError(f"tracker {device.pika_tracker_device!r} has no fresh pose")
            distance = sense.get_gripper_distance()
            command = sense.get_command_state()
            result = {
                "port": str(teleop["port"]),
                "tracker": device.pika_tracker_device,
                "position_m": [round(float(value), 6) for value in pose.position],
                "gripper_mm": None if distance is None else round(float(distance), 3),
                "command_state": None if command is None else int(command),
            }
            report.add("ok", f"Pika {side} live", json.dumps(result, ensure_ascii=False))
        except Exception as exc:
            report.add("fail", f"Pika {side} live", f"{type(exc).__name__}: {exc}")

    for side, device in reversed(list(devices.items())):
        try:
            device.disconnect()
        except Exception as exc:
            report.add("warn", f"Pika {side} close", str(exc))


def show_kernel_usb_log(report: Report) -> None:
    print("\n== Recent kernel USB/CAN log ==")
    code, output = run_command(
        ["journalctl", "-k", "--since", "-10 min", "--no-pager"], timeout_s=10.0
    )
    if code != 0:
        report.add("warn", "Kernel log", output)
        return
    keywords = (
        "usb",
        "uvc",
        "video4linux",
        "ttyusb",
        "ttyacm",
        "can_left",
        "can_right",
        "bandwidth",
        "over-current",
    )
    lines = [line for line in output.splitlines() if any(word in line.lower() for word in keywords)]
    selected = "\n".join(lines[-120:])
    report.add("info", "Kernel log", "\n    " + selected.replace("\n", "\n    ") if selected else "no matching lines")
    suspicious = ("not enough bandwidth", "over-current", "error -71", "error -110", "device descriptor read")
    matches = [line for line in lines if any(word in line.lower() for word in suspicious)]
    if matches:
        report.add("warn", "USB errors", "\n    " + "\n    ".join(matches[-20:]))


def print_conclusion(report: Report, live: bool) -> None:
    print("\n== Conclusion ==")
    print(
        f"FAIL={report.count('fail')} WARN={report.count('warn')} "
        f"OK={report.count('ok')}"
    )
    if report.failed:
        print("Hardware is not ready. Fix the first FAIL item, then rerun this command.")
    elif not live:
        print("Enumeration passed. Run again with --live to verify actual Pika/Piper feedback.")
    else:
        print("Enumeration and live feedback passed. The Pika/Piper hardware is ready for teleop preflight.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Scan three D435i cameras plus the configured dual Pika/Piper hardware."
    )
    parser.add_argument("config", nargs="?", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--expected-realsense",
        type=int,
        default=3,
        help="expected number of RealSense devices (default: 3)",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="open Pika serial and Piper CAN devices to read feedback (no commands sent)",
    )
    parser.add_argument(
        "--left-can",
        help="temporarily override the configured left Piper CAN interface for this scan",
    )
    parser.add_argument(
        "--right-can",
        help="temporarily override the configured right Piper CAN interface for this scan",
    )
    parser.add_argument(
        "--kernel-log",
        action="store_true",
        help="include relevant kernel messages from the last ten minutes",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = Report()
    print("Pika / Piper / RealSense diagnostic")
    print(f"host={platform.node()} os={platform.platform()} python={sys.executable}")
    print(f"config={args.config.resolve()}")
    print("robot_commands_sent=false")

    if platform.system() != "Linux":
        report.add("fail", "Operating system", "this hardware scan must run on the Ubuntu robot host")
        print_conclusion(report, args.live)
        return 2

    try:
        config = load_config(args.config)
    except Exception as exc:
        report.add("fail", "Configuration", f"{type(exc).__name__}: {exc}")
        print_conclusion(report, args.live)
        return 2

    if args.left_can:
        config["robot"]["robots"]["left"]["port"] = args.left_can
        report.add("info", "Left CAN override", args.left_can)
    if args.right_can:
        config["robot"]["robots"]["right"]["port"] = args.right_can
        report.add("info", "Right CAN override", args.right_can)

    scan_realsense(args.expected_realsense, report)
    scan_expected_paths(config, report)
    scan_can(config, report)
    if args.kernel_log:
        show_kernel_usb_log(report)
    if args.live:
        probe_pipers(config, report)
        probe_pikas(config, report)
    print_conclusion(report, args.live)
    return 1 if report.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
