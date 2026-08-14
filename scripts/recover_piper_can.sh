#!/usr/bin/env bash
# Recover one SocketCAN/gs_usb interface without sending robot commands.
set -euo pipefail

readonly INTERFACE="${1:-can_left}"
readonly MODE="${2:---soft}"
readonly CAN_BITRATE=1000000
readonly CAN_RESTART_MS=100
readonly CAN_TX_QUEUE_LEN=1000

if ((EUID != 0)); then
    echo "Run with sudo: sudo $0 ${INTERFACE} [--soft|--usb-reset]" >&2
    exit 2
fi

if [[ ! "${INTERFACE}" =~ ^[a-zA-Z0-9_.-]+$ ]]; then
    echo "Invalid interface name: ${INTERFACE}" >&2
    exit 2
fi

if [[ "${MODE}" != "--soft" && "${MODE}" != "--usb-reset" ]]; then
    echo "Usage: sudo $0 [interface] [--soft|--usb-reset]" >&2
    exit 2
fi

if [[ ! -e "/sys/class/net/${INTERFACE}" ]]; then
    echo "CAN interface does not exist: ${INTERFACE}" >&2
    /usr/sbin/ip -brief link show type can || true
    exit 1
fi

readonly DEVICE_PATH="$(readlink -f "/sys/class/net/${INTERFACE}/device")"
readonly DRIVER_PATH="$(readlink -f "/sys/class/net/${INTERFACE}/device/driver")"
readonly DRIVER_NAME="$(basename "${DRIVER_PATH}")"
readonly USB_INTERFACE="$(basename "${DEVICE_PATH}")"

echo "Target interface : ${INTERFACE}"
echo "Device path      : ${DEVICE_PATH}"
echo "Driver           : ${DRIVER_NAME}"
echo "USB interface    : ${USB_INTERFACE}"
echo "Robot commands   : false"

if [[ "${DRIVER_NAME}" != "gs_usb" ]]; then
    echo "Refusing recovery: ${INTERFACE} is not driven by gs_usb." >&2
    exit 1
fi

configure_can() {
    local interface="$1"
    /usr/sbin/ip link set dev "${interface}" down || true
    if ! /usr/sbin/ip link set dev "${interface}" type can \
        bitrate "${CAN_BITRATE}" restart-ms "${CAN_RESTART_MS}"; then
        echo "${interface} does not support restart-ms; configuring bitrate only." >&2
        /usr/sbin/ip link set dev "${interface}" type can bitrate "${CAN_BITRATE}"
    fi
    /usr/sbin/ip link set dev "${interface}" txqueuelen "${CAN_TX_QUEUE_LEN}"
    /usr/sbin/ip link set dev "${interface}" up
}

if [[ "${MODE}" == "--usb-reset" ]]; then
    echo "Rebinding only ${USB_INTERFACE} through ${DRIVER_PATH}."
    printf '%s' "${USB_INTERFACE}" >"${DRIVER_PATH}/unbind"
    sleep 1
    printf '%s' "${USB_INTERFACE}" >"${DRIVER_PATH}/bind"

    for _ in $(seq 1 50); do
        [[ -e "/sys/class/net/${INTERFACE}" ]] && break
        sleep 0.2
    done
    if [[ ! -e "/sys/class/net/${INTERFACE}" ]]; then
        echo "${INTERFACE} did not return after the USB-driver rebind." >&2
        echo "Available CAN interfaces:" >&2
        /usr/sbin/ip -brief link show type can >&2 || true
        echo "Check the udev stable-name rule for USB interface ${USB_INTERFACE}." >&2
        exit 1
    fi
fi

configure_can "${INTERFACE}"
/usr/sbin/ip -details -statistics link show dev "${INTERFACE}"

if ! command -v candump >/dev/null 2>&1; then
    echo "WARN: candump is unavailable; install can-utils to test traffic."
    exit 0
fi

echo
echo "Listening for CAN frames for 5 seconds..."
set +e
CAN_OUTPUT="$(timeout 5 candump -L "${INTERFACE}" 2>&1)"
CAN_STATUS=$?
set -e

if [[ -n "${CAN_OUTPUT}" ]]; then
    printf '%s\n' "${CAN_OUTPUT}" | head -n 30
    echo "PASS: ${INTERFACE} is receiving CAN frames."
    exit 0
fi

if ((CAN_STATUS == 124)); then
    echo "FAIL: ${INTERFACE} received no CAN frames in 5 seconds."
    echo "The SocketCAN adapter is configured, but the attached Piper bus is silent."
    exit 1
fi

echo "FAIL: candump exited with status ${CAN_STATUS}."
exit 1
