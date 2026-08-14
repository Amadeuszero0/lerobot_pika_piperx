import logging
from typing import Any


def disconnect_devices(devices: list[Any], *, suppress_errors: bool) -> None:
    """Disconnect every constructed device, preserving the first cleanup error."""
    first_error: Exception | None = None
    for device in devices:
        try:
            device.disconnect()
        except Exception as exc:
            logging.exception("Failed to disconnect %s", device)
            first_error = first_error or exc
    if first_error is not None and not suppress_errors:
        raise first_error
