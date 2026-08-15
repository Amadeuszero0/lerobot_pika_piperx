#!/usr/bin/env bash
set -euo pipefail

readonly REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ENV_BIN="/home/star/miniconda3/envs/lerobot_real/bin"
readonly BASE_CONFIG="${PIPER_CONFIG_PATH:-${REPO_ROOT}/config/piper/dual_pika_piper_local.yaml}"
readonly BINDINGS="${D435I_BINDINGS_PATH:-${REPO_ROOT}/config/piper/d435i_roles_local.yaml}"
readonly RECORD_CONFIG="${PIPER_RECORD_CONFIG_PATH:-${REPO_ROOT}/config/piper/dual_pika_piper_record_production_local.yaml}"

session_name="dual_pika_piper"
task_description="Bimanual Pika to Piper teleoperation"
num_episodes=50
episode_time_s=30
episode_mode="timed"
interactive_reset=true
reset_time_s=20
move_speed_percent=40
dataset_base="/home/star/lerobot_data"
preflight_seconds=10
resume_dataset=""

usage() {
    cat <<'EOF'
Usage:
  bash scripts/start_dual_pika_piper_recording.sh [options]

Options:
  --session NAME            Dataset name prefix (default: dual_pika_piper)
  --task TEXT               LeRobot task description
  --episodes N              Number of episodes (default: 50); with --resume,
                            this is the desired total including saved episodes
  --episode-seconds N       Maximum duration of each episode (default: 30)
  --timed-episodes          Use the official-style maximum duration (default)
  --manual-episodes         Disable the B-recording time limit
  --interactive-reset       Teleoperate without recording to restore the A-end
                            state before every episode (default)
  --no-interactive-reset    Skip PREP and enter B recording directly
  --reset-seconds N         Reset interval in config (default: 20)
  --speed N                 Piper speed percentage, 1-100 (default: 40)
  --dataset-base PATH       Parent data directory (default: /home/star/lerobot_data)
  --preflight-seconds N     Simultaneous camera test duration (default: 10)
  --resume PATH             Append to this existing LeRobot dataset instead of
                            creating a timestamped dataset
  -h, --help                Show this help

Every episode moves both Pipers to their configured startup poses, then waits
for a close-open-close gesture from both Pika grippers before teleoperation.

Headless terminal workflow:
  Preparation: Enter starts B recording; q+Enter quits. No data is written.
  B recording: Enter or s+Enter finishes early; r+Enter discards and returns
               to preparation; q+Enter discards and quits.
  Review:      s saves; r discards and returns to preparation; q discards/quits.
  Timed mode:  reaching --episode-seconds opens the same review prompt.
  After save:  Enter starts the next unrecorded preparation phase; q stops.

Saving is synchronous, matching LeRobot: the next-episode prompt appears only
after the current episode's Parquet, video, and metadata files are saved.

Resume example (continue the dataset until it contains 50 saved episodes):
  bash scripts/collect_dual_pika_piper_dataset.sh \
    --resume /home/star/lerobot_data/dual_pika_piper_dataset_YYYYMMDD_HHMMSS \
    --episodes 50
EOF
}

require_value() {
    if [[ "$#" -lt 2 || -z "${2:-}" ]]; then
        echo "Missing value for $1" >&2
        usage >&2
        exit 2
    fi
}

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --session)
            require_value "$@"; session_name="$2"; shift 2 ;;
        --task)
            require_value "$@"; task_description="$2"; shift 2 ;;
        --episodes)
            require_value "$@"; num_episodes="$2"; shift 2 ;;
        --episode-seconds)
            require_value "$@"; episode_time_s="$2"; shift 2 ;;
        --manual-episodes)
            episode_mode="manual"; shift ;;
        --timed-episodes)
            episode_mode="timed"; shift ;;
        --interactive-reset)
            interactive_reset=true; shift ;;
        --no-interactive-reset)
            interactive_reset=false; shift ;;
        --reset-seconds)
            require_value "$@"; reset_time_s="$2"; shift 2 ;;
        --speed)
            require_value "$@"; move_speed_percent="$2"; shift 2 ;;
        --dataset-base)
            require_value "$@"; dataset_base="$2"; shift 2 ;;
        --preflight-seconds)
            require_value "$@"; preflight_seconds="$2"; shift 2 ;;
        --resume)
            require_value "$@"; resume_dataset="$2"; shift 2 ;;
        -h|--help)
            usage; exit 0 ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2 ;;
    esac
done

if [[ ! "${session_name}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
    echo "--session may contain only letters, numbers, dot, underscore, and hyphen" >&2
    exit 2
fi
if [[ ! "${num_episodes}" =~ ^[1-9][0-9]*$ ]]; then
    echo "--episodes must be a positive integer" >&2
    exit 2
fi
if [[ ! "${episode_time_s}" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ "${episode_time_s}" == "0" ]]; then
    echo "--episode-seconds must be positive" >&2
    exit 2
fi
if [[ ! "${reset_time_s}" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ "${reset_time_s}" == "0" ]]; then
    echo "--reset-seconds must be positive" >&2
    exit 2
fi
if [[ ! "${move_speed_percent}" =~ ^[0-9]+$ ]] \
    || (( move_speed_percent < 1 || move_speed_percent > 100 )); then
    echo "--speed must be an integer from 1 to 100" >&2
    exit 2
fi
if [[ ! "${preflight_seconds}" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ "${preflight_seconds}" == "0" ]]; then
    echo "--preflight-seconds must be positive" >&2
    exit 2
fi

for path in \
    "${ENV_BIN}/python" \
    "${ENV_BIN}/lerobot-real-piper-check-config" \
    "${ENV_BIN}/lerobot-real-record" \
    "${BASE_CONFIG}" \
    "${BINDINGS}" \
    "${REPO_ROOT}/scripts/prepare_dual_pika_piper_recording.py" \
    "${REPO_ROOT}/scripts/inspect_lerobot_resume.py" \
    "${REPO_ROOT}/scripts/check_dual_pika_piper_hardware.py" \
    /dev/pika_left \
    /dev/pika_right; do
    if [[ ! -e "${path}" ]]; then
        echo "Required path is missing: ${path}" >&2
        exit 1
    fi
done

run_mode="NEW"
saved_episodes=0
episodes_this_run="${num_episodes}"
if [[ -n "${resume_dataset}" ]]; then
    run_mode="RESUME"
    resume_plan="$(
        "${ENV_BIN}/python" "${REPO_ROOT}/scripts/inspect_lerobot_resume.py" \
            "${resume_dataset}" \
            --target-episodes "${num_episodes}"
    )"
    IFS=$'\t' read -r DATASET_ROOT REPO_ID saved_episodes episodes_this_run \
        <<<"${resume_plan}"
    if [[ -z "${DATASET_ROOT}" || -z "${REPO_ID}" \
        || -z "${saved_episodes}" || -z "${episodes_this_run}" ]]; then
        echo "Resume planner returned incomplete data" >&2
        exit 1
    fi
    RUN_NAME="${REPO_ID#local/}"
else
    RUN_TAG="$(date +%Y%m%d_%H%M%S)"
    RUN_NAME="${session_name}_${RUN_TAG}"
    DATASET_ROOT="${dataset_base%/}/${RUN_NAME}"
    REPO_ID="local/${RUN_NAME}"

    if [[ -e "${DATASET_ROOT}" ]]; then
        echo "Refusing to overwrite existing dataset: ${DATASET_ROOT}" >&2
        exit 1
    fi
fi
readonly run_mode saved_episodes episodes_this_run RUN_NAME DATASET_ROOT REPO_ID

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

echo "===== Camera preflight ====="
"${ENV_BIN}/python" "${REPO_ROOT}/scripts/prepare_dual_pika_piper_recording.py" \
    --bindings "${BINDINGS}" preflight --seconds "${preflight_seconds}"

echo "===== Generate production config ====="
"${ENV_BIN}/python" "${REPO_ROOT}/scripts/prepare_dual_pika_piper_recording.py" \
    --bindings "${BINDINGS}" generate \
    --base-config "${BASE_CONFIG}" \
    --output "${RECORD_CONFIG}" \
    --dataset-root "${DATASET_ROOT}" \
    --repo-id "${REPO_ID}" \
    --num-episodes "${episodes_this_run}" \
    --episode-time-s "${episode_time_s}" \
    --reset-time-s "${reset_time_s}" \
    --single-task "${task_description}" \
    --move-speed-percent "${move_speed_percent}"

echo "===== Runtime config validation ====="
"${ENV_BIN}/lerobot-real-piper-check-config" "${RECORD_CONFIG}"
"${ENV_BIN}/lerobot-real-record" \
    --check-config-only \
    --config_path "${RECORD_CONFIG}"

echo "===== Pika and Piper read-only preflight ====="
"${ENV_BIN}/python" \
    "${REPO_ROOT}/scripts/check_dual_pika_piper_hardware.py" \
    "${RECORD_CONFIG}"

cat <<EOF

===== Production recording ready =====
Mode:               ${run_mode}
Dataset root:       ${DATASET_ROOT}
LeRobot repo_id:    ${REPO_ID}
Task:               ${task_description}
Target episodes:    ${num_episodes}
Already saved:      ${saved_episodes}
Episodes this run:  ${episodes_this_run}
Episode mode:       ${episode_mode}
Episode duration:   $([[ "${episode_mode}" == "manual" ]] && echo "manual (no time limit)" || echo "maximum ${episode_time_s} s")
Between-episode A-end restoration: ${interactive_reset}
Piper speed:        ${move_speed_percent}%
Startup motion:     enabled
Camera streams:     left.third_view, left.wrist, right.wrist
Joint observations: left/right joint1..joint6.angle_rad
Terminal controls:  prep Enter=start B; record Enter=finish/review;
                    review s=save, r=discard/re-prepare, q=discard/quit

Both Pipers will move before each episode. Clear both workspaces and prepare E-stop.
EOF

required_confirmation="RECORD"
if [[ "${run_mode}" == "RESUME" ]]; then
    required_confirmation="RESUME"
fi
read -r -p "Type ${required_confirmation} to start the production session: " confirmation
if [[ "${confirmation}" != "${required_confirmation}" ]]; then
    echo "Cancelled. No recording command was sent."
    exit 1
fi

record_args=(--config_path "${RECORD_CONFIG}")
if [[ "${episode_mode}" == "manual" ]]; then
    record_args=(--manual-episode-control "${record_args[@]}")
fi
if [[ "${interactive_reset}" == "true" ]]; then
    record_args=(--interactive-episode-reset "${record_args[@]}")
fi
if [[ "${run_mode}" == "RESUME" ]]; then
    record_args=(--resume "${record_args[@]}")
fi

set +e
"${ENV_BIN}/lerobot-real-record" "${record_args[@]}"
record_status=$?
set -e

echo "Dataset root: ${DATASET_ROOT}"
if [[ "${record_status}" -eq 0 ]]; then
    echo "recording_status=PASS"
else
    echo "recording_status=FAILED(${record_status})" >&2
fi
exit "${record_status}"
