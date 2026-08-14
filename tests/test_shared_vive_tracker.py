from types import SimpleNamespace

from lerobot_real.devices.pika import shared_vive_tracker as shared_module
from lerobot_real.devices.pika.shared_vive_tracker import SharedViveTracker


class _Device:
    def __init__(self, name: str, serial: str) -> None:
        self._name = name
        self.ptr = serial

    def Name(self) -> bytes:
        return self._name.encode()


class _Context:
    def __init__(self) -> None:
        self.devices = [_Device("WM0", "LHR-TRACKER")]

    def Objects(self) -> list[_Device]:
        return self.devices


class _Tracker:
    def __init__(self) -> None:
        self.context = None
        self.running = False
        self.poses = {"WM0": object()}


class _OfficialMethods:
    def __init__(self) -> None:
        self.connect_count = 0
        self.disconnect_count = 0
        self.pose_requests: list[str | None] = []

    def connect(self, tracker: _Tracker) -> bool:
        self.connect_count += 1
        tracker.context = _Context()
        tracker.running = True
        return True

    def disconnect(self, tracker: _Tracker) -> None:
        self.disconnect_count += 1
        tracker.running = False
        tracker.context = None

    def get_pose(self, tracker: _Tracker, device: str | None = None):
        self.pose_requests.append(device)
        if device is None:
            return tracker.poses.copy()
        return tracker.poses.get(device)

    @staticmethod
    def get_devices(tracker: _Tracker) -> list[str]:
        return [device.Name().decode() for device in tracker.context.Objects()]

    @staticmethod
    def get_device_info(tracker: _Tracker, device: str | None = None):
        info = {"WM0": {"updates": 1}}
        return info.get(device) if device else info


def _attach(
    shared: SharedViveTracker, tracker: _Tracker, official: _OfficialMethods
) -> bool:
    return shared.attach(
        tracker,
        connect=official.connect,
        disconnect=official.disconnect,
        get_pose=official.get_pose,
        get_devices=official.get_devices,
        get_device_info=official.get_device_info,
    )


def test_two_clients_share_the_official_sdk_collector(monkeypatch) -> None:
    monkeypatch.setattr(
        shared_module,
        "pysurvive",
        SimpleNamespace(simple_serial_number=lambda pointer: pointer),
    )
    shared = SharedViveTracker()
    official = _OfficialMethods()
    owner = _Tracker()
    follower = _Tracker()

    assert _attach(shared, owner, official)
    assert _attach(shared, follower, official)

    assert official.connect_count == 1
    assert follower.context is owner.context
    assert shared.get_pose("LHR-TRACKER") is owner.poses["WM0"]
    assert official.pose_requests == ["WM0"]
    assert shared.tracker_identities() == {"WM0": "LHR-TRACKER"}


def test_official_collector_stays_alive_until_last_client_detaches(monkeypatch) -> None:
    monkeypatch.setattr(
        shared_module,
        "pysurvive",
        SimpleNamespace(simple_serial_number=lambda pointer: pointer),
    )
    shared = SharedViveTracker()
    official = _OfficialMethods()
    owner = _Tracker()
    follower = _Tracker()
    _attach(shared, owner, official)
    _attach(shared, follower, official)

    shared.detach(owner)
    assert owner.running
    assert official.disconnect_count == 0

    shared.detach(follower)
    assert not owner.running
    assert official.disconnect_count == 1
