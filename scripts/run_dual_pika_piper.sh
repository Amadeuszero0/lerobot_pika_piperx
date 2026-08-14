#!/usr/bin/env bash
set -euo pipefail

readonly REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ENV_BIN="/home/star/miniconda3/envs/lerobot_real/bin"
readonly CONFIG_PATH="${PIPER_CONFIG_PATH:-${REPO_ROOT}/config/piper/dual_pika_piper_local.yaml}"
readonly MODE="${1:---check}"

if [[ "${MODE}" != "--check" && "${MODE}" != "--run" ]]; then
    echo "Usage: $0 [--check|--run]" >&2
    exit 2
fi

for path in \
    "${ENV_BIN}/lerobot-real-piper-check-config" \
    "${ENV_BIN}/lerobot-real-teleop" \
    "${CONFIG_PATH}" \
    "${REPO_ROOT}/scripts/check_dual_pika_piper_hardware.py" \
    /dev/pika_left \
    /dev/pika_right; do
    if [[ ! -e "${path}" ]]; then
        echo "Required path is missing: ${path}" >&2
        exit 1
    fi
done

for interface in can_left can_right; do
    if ! /usr/sbin/ip link show dev "${interface}" >/dev/null 2>&1; then
        echo "Missing CAN interface: ${interface}" >&2
        exit 1
    fi
    if ! /usr/sbin/ip link show dev "${interface}" | /usr/bin/grep -q "UP"; then
        echo "CAN interface is not up: ${interface}" >&2
        echo "Run sudo ./scripts/setup_piper_can.sh when you are ready to configure it." >&2
        exit 1
    fi
done

"${ENV_BIN}/lerobot-real-piper-check-config" "${CONFIG_PATH}"
"${ENV_BIN}/python" \
    "${REPO_ROOT}/scripts/check_dual_pika_piper_hardware.py" \
    "${CONFIG_PATH}"
echo "Dual-Piper preflight passed. No robot command was sent."

if [[ "${MODE}" == "--check" ]]; then
    exit 0
fi

echo "The next process connects both Piper arms and enables torque."
echo "It remains paused until the teleoperation prompt is confirmed."
read -r -p "Type RUN after clearing both workspaces: " confirmation
if [[ "${confirmation}" != "RUN" ]]; then
    echo "Cancelled."
    exit 1
fi

exec "${ENV_BIN}/lerobot-real-teleop" --config_path "${CONFIG_PATH}"
