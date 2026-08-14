#!/usr/bin/env bash
set -euo pipefail

if ((EUID != 0)); then
    echo "Run this script with sudo." >&2
    exit 1
fi

readonly CAN_BITRATE=1000000
readonly CAN_RESTART_MS=100
readonly CAN_TX_QUEUE_LEN=1000

if (($# == 0)); then
    interfaces=(can_left can_right)
else
    interfaces=("$@")
fi

for interface in "${interfaces[@]}"; do
    if ! /usr/sbin/ip link show dev "${interface}" >/dev/null 2>&1; then
        echo "Piper CAN interface ${interface} was not found." >&2
        exit 1
    fi

    /usr/sbin/ip link set dev "${interface}" down
    if ! /usr/sbin/ip link set dev "${interface}" type can \
        bitrate "${CAN_BITRATE}" restart-ms "${CAN_RESTART_MS}"; then
        echo "${interface} does not support restart-ms; configuring bitrate only." >&2
        /usr/sbin/ip link set dev "${interface}" type can bitrate "${CAN_BITRATE}"
    fi
    /usr/sbin/ip link set dev "${interface}" txqueuelen "${CAN_TX_QUEUE_LEN}"
    /usr/sbin/ip link set dev "${interface}" up
    echo "Configured ${interface}: ${CAN_BITRATE} bit/s"
done
