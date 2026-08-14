#!/usr/bin/env bash
set -euo pipefail

readonly REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

exec bash "${REPO_ROOT}/scripts/start_dual_pika_piper_recording.sh" \
    --session practice_dual_pika_piper \
    --task "Bimanual Pika to Piper practice" \
    --episodes 5 \
    --episode-seconds 30 \
    "$@"
