#!/usr/bin/env bash
set -euo pipefail

readonly REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ENV_BIN="/home/star/miniconda3/envs/lerobot_real/bin"
readonly BASE_CONFIG="${PIPER_CONFIG_PATH:-${REPO_ROOT}/config/piper/dual_pika_piper_local.yaml}"
readonly BINDINGS="${D435I_BINDINGS_PATH:-${REPO_ROOT}/config/piper/d435i_roles_local.yaml}"
readonly RECORD_CONFIG="${PIPER_RECORD_CONFIG_PATH:-${REPO_ROOT}/config/piper/dual_pika_piper_record_local.yaml}"
readonly MODE="${1:---check}"

if [[ "$#" -gt 0 ]]; then
    shift
fi

if [[ "$#" -ne 0 ]]; then
    echo "Unexpected extra arguments: $*" >&2
    echo "Use --smoke for the built-in 10-second single-episode test." >&2
    exit 2
fi

if [[ "${MODE}" != "--check" && "${MODE}" != "--run" && "${MODE}" != "--smoke" ]]; then
    echo "Usage: $0 [--check|--run|--smoke]" >&2
    exit 2
fi

GENERATE_ARGS=()
if [[ "${MODE}" == "--smoke" ]]; then
    readonly SMOKE_TAG="$(date +%Y%m%d_%H%M%S)"
    readonly SMOKE_ROOT="/home/star/lerobot_data/dual_pika_piper_smoke_${SMOKE_TAG}"
    GENERATE_ARGS=(
        --dataset-root "${SMOKE_ROOT}"
        --repo-id "local/dual_pika_piper_smoke_${SMOKE_TAG}"
        --num-episodes 1
        --episode-time-s 10
        --reset-time-s 3
        --move-speed-percent 10
    )
fi

for path in \
    "${ENV_BIN}/python" \
    "${ENV_BIN}/lerobot-real-piper-check-config" \
    "${ENV_BIN}/lerobot-real-record" \
    "${BASE_CONFIG}" \
    "${BINDINGS}" \
    "${REPO_ROOT}/scripts/prepare_dual_pika_piper_recording.py" \
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
        exit 1
    fi
done

"${ENV_BIN}/python" "${REPO_ROOT}/scripts/prepare_dual_pika_piper_recording.py" \
    --bindings "${BINDINGS}" preflight --seconds 10

"${ENV_BIN}/python" "${REPO_ROOT}/scripts/prepare_dual_pika_piper_recording.py" \
    --bindings "${BINDINGS}" generate \
    --base-config "${BASE_CONFIG}" \
    --output "${RECORD_CONFIG}" \
    "${GENERATE_ARGS[@]}"

"${ENV_BIN}/lerobot-real-piper-check-config" "${RECORD_CONFIG}"
"${ENV_BIN}/lerobot-real-record" \
    --check-config-only \
    --config_path "${RECORD_CONFIG}"
"${ENV_BIN}/python" \
    "${REPO_ROOT}/scripts/check_dual_pika_piper_hardware.py" \
    "${RECORD_CONFIG}"

echo "Camera, Pika, and Piper preflight passed. No robot command was sent."
if [[ "${MODE}" == "--check" ]]; then
    exit 0
fi

echo "The next process sets both Pipers to follower mode, enables torque,"
echo "opens all three RGB cameras, and starts LeRobot recording."
if [[ "${MODE}" == "--smoke" ]]; then
    echo "Smoke mode: one 10-second episode at 10% speed."
    echo "Both Pipers will move to their configured startup poses before Pika activation."
    echo "Dataset root: ${SMOKE_ROOT}"
fi
read -r -p "Clear both workspaces and type RECORD to continue: " confirmation
if [[ "${confirmation}" != "RECORD" ]]; then
    echo "Cancelled."
    exit 1
fi

exec "${ENV_BIN}/lerobot-real-record" \
    --config_path "${RECORD_CONFIG}"
