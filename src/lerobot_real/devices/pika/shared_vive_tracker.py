"""Share one official Pika SDK Vive tracker across multiple Sense devices.

The Pika SDK normally creates a libsurvive context for every ``Sense``
instance. Two contexts cannot reliably own the same Vive USB devices. This
module keeps one SDK ``ViveTracker`` as the owner and proxies every additional
tracker object to it. Pose collection and tracker-to-gripper conversion remain
entirely inside the official Pika SDK.
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable
from typing import Any

try:
    import pysurvive
except ImportError:  # pragma: no cover - Pika support is optional
    pysurvive = None


logger = logging.getLogger(__name__)


class SharedViveTracker:
    """Own one official SDK tracker and route all Pika clients through it."""

    _instance: SharedViveTracker | None = None
    _instance_lock = threading.Lock()

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._owner: Any | None = None
        self._clients: set[Any] = set()
        self._serial_by_name: dict[str, str] = {}
        self._name_by_serial: dict[str, str] = {}
        self._connect: Callable[[Any], bool] | None = None
        self._disconnect: Callable[[Any], None] | None = None
        self._get_pose: Callable[..., Any] | None = None
        self._get_devices: Callable[[Any], list[str]] | None = None
        self._get_device_info: Callable[..., Any] | None = None

    @classmethod
    def instance(cls) -> SharedViveTracker:
        if cls._instance is None:
            with cls._instance_lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    @property
    def ctx(self) -> Any | None:
        with self._lock:
            return None if self._owner is None else self._owner.context

    def attach(
        self,
        tracker: Any,
        *,
        connect: Callable[[Any], bool],
        disconnect: Callable[[Any], None],
        get_pose: Callable[..., Any],
        get_devices: Callable[[Any], list[str]],
        get_device_info: Callable[..., Any],
    ) -> bool:
        """Attach an SDK tracker, starting the official collector only once."""
        with self._lock:
            if tracker in self._clients:
                return True

            if self._owner is None:
                self._owner = tracker
                self._connect = connect
                self._disconnect = disconnect
                self._get_pose = get_pose
                self._get_devices = get_devices
                self._get_device_info = get_device_info
                try:
                    connected = bool(connect(tracker))
                except BaseException:
                    self._clear_locked()
                    raise
                if not connected:
                    self._clear_locked()
                    return False
            else:
                tracker.context = self._owner.context
                tracker.running = True

            tracker._lerobot_real_shared = self
            self._clients.add(tracker)

        self._refresh_identities()
        return True

    def get_pose(self, device: str | None = None) -> Any:
        owner, callback = self._owner_callback(self._get_pose)
        if owner is None or callback is None:
            return None if device else {}
        resolved = self._resolve_device(device) if device else None
        return callback(owner, resolved)

    def get_devices(self) -> list[str]:
        owner, callback = self._owner_callback(self._get_devices)
        if owner is None or callback is None:
            return []
        self._refresh_identities()
        return callback(owner)

    def get_device_info(self, device: str | None = None) -> Any:
        owner, callback = self._owner_callback(self._get_device_info)
        if owner is None or callback is None:
            return None if device else {}
        resolved = self._resolve_device(device) if device else None
        return callback(owner, resolved)

    def detach(self, tracker: Any) -> None:
        owner: Any | None = None
        disconnect: Callable[[Any], None] | None = None
        with self._lock:
            if tracker not in self._clients:
                tracker.running = False
                return
            self._clients.remove(tracker)
            if self._clients:
                # The owner object must remain alive while its official SDK
                # collector threads serve the other attached tracker objects.
                if tracker is not self._owner:
                    tracker.running = False
                    tracker.context = None
                return

            owner = self._owner
            disconnect = self._disconnect
            self._clear_locked()

        if owner is not None and disconnect is not None:
            disconnect(owner)

    def tracker_serials(self) -> list[str]:
        self._refresh_identities()
        with self._lock:
            return sorted(
                serial
                for name, serial in self._serial_by_name.items()
                if not name.startswith("LH")
            )

    def tracker_identities(self) -> dict[str, str]:
        self._refresh_identities()
        with self._lock:
            return {
                name: serial
                for name, serial in self._serial_by_name.items()
                if not name.startswith("LH")
            }

    def shutdown(self) -> None:
        owner: Any | None = None
        disconnect: Callable[[Any], None] | None = None
        with self._lock:
            owner = self._owner
            disconnect = self._disconnect
            clients = tuple(self._clients)
            self._clear_locked()
            for tracker in clients:
                if tracker is not owner:
                    tracker.running = False
                    tracker.context = None
        if owner is not None and disconnect is not None:
            disconnect(owner)

    def _owner_callback(self, callback: Callable[..., Any] | None) -> tuple[Any, Any]:
        with self._lock:
            return self._owner, callback

    def _resolve_device(self, device: str) -> str:
        self._refresh_identities()
        with self._lock:
            return self._name_by_serial.get(device, device)

    def _refresh_identities(self) -> None:
        if pysurvive is None:
            return
        with self._lock:
            owner = self._owner
        context = None if owner is None else getattr(owner, "context", None)
        serial_number = getattr(pysurvive, "simple_serial_number", None)
        if context is None or serial_number is None:
            return

        pointers_by_name: dict[str, Any] = {}
        get_object = getattr(pysurvive, "simple_get_object", None)
        context_pointer = getattr(context, "ptr", None)
        data_lock = getattr(owner, "data_lock", None)
        devices_info = getattr(owner, "devices_info", None)
        if get_object is not None and context_pointer is not None and data_lock is not None:
            try:
                with data_lock:
                    names = tuple(devices_info) if devices_info is not None else ()
                for name in names:
                    pointer = get_object(context_pointer, name.encode())
                    if pointer:
                        pointers_by_name[name] = pointer
            except Exception:
                logger.debug("Could not resolve dynamic Vive devices", exc_info=True)

        # Some pysurvive builds expose a static context.Objects() snapshot. It
        # is still useful for tests and for devices present during startup.
        if not pointers_by_name:
            try:
                for device in context.Objects():
                    name = device.Name()
                    if isinstance(name, bytes):
                        name = name.decode("utf-8", errors="replace")
                    pointer = getattr(device, "ptr", None)
                    if pointer is not None:
                        pointers_by_name[str(name).strip()] = pointer
            except Exception:
                logger.debug("Could not enumerate shared Vive devices", exc_info=True)

        discovered: dict[str, str] = {}
        for name, pointer in pointers_by_name.items():
            try:
                serial = serial_number(pointer)
                if isinstance(serial, bytes):
                    serial = serial.decode("utf-8", errors="replace")
                name = str(name).strip()
                serial = str(serial).strip()
                if name and serial:
                    discovered[name] = serial
            except Exception:
                logger.debug("Could not identify a shared Vive device", exc_info=True)

        if discovered:
            with self._lock:
                self._serial_by_name.update(discovered)
                self._name_by_serial.update(
                    {serial: name for name, serial in discovered.items()}
                )

    def _clear_locked(self) -> None:
        self._owner = None
        self._clients.clear()
        self._serial_by_name.clear()
        self._name_by_serial.clear()
        self._connect = None
        self._disconnect = None
        self._get_pose = None
        self._get_devices = None
        self._get_device_info = None


def _install_shared_tracker_patch() -> None:
    if pysurvive is None:
        return
    try:
        from pika.tracker import vive_tracker as vive_tracker_module
    except ImportError:
        return

    tracker_class = getattr(vive_tracker_module, "ViveTracker", None)
    if tracker_class is None or getattr(
        tracker_class, "_lerobot_real_shared_patched", False
    ):
        return

    original_connect = tracker_class.connect
    original_disconnect = tracker_class.disconnect
    original_get_pose = tracker_class.get_pose
    original_get_devices = tracker_class.get_devices
    original_get_device_info = tracker_class.get_device_info

    def connect(self) -> bool:
        return SharedViveTracker.instance().attach(
            self,
            connect=original_connect,
            disconnect=original_disconnect,
            get_pose=original_get_pose,
            get_devices=original_get_devices,
            get_device_info=original_get_device_info,
        )

    def disconnect(self) -> None:
        shared = getattr(self, "_lerobot_real_shared", None)
        if shared is None:
            return original_disconnect(self)
        shared.detach(self)

    def get_pose(self, device: str | None = None):
        shared = getattr(self, "_lerobot_real_shared", None)
        if shared is None:
            return original_get_pose(self, device)
        return shared.get_pose(device)

    def get_devices(self):
        shared = getattr(self, "_lerobot_real_shared", None)
        if shared is None:
            return original_get_devices(self)
        return shared.get_devices()

    def get_device_info(self, device: str | None = None):
        shared = getattr(self, "_lerobot_real_shared", None)
        if shared is None:
            return original_get_device_info(self, device)
        return shared.get_device_info(device)

    tracker_class.connect = connect
    tracker_class.disconnect = disconnect
    tracker_class.get_pose = get_pose
    tracker_class.get_devices = get_devices
    tracker_class.get_device_info = get_device_info
    tracker_class._lerobot_real_shared_patched = True


_install_shared_tracker_patch()

__all__ = ["SharedViveTracker"]
