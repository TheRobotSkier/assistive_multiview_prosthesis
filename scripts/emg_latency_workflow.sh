#!/bin/bash
# EMG collection + training + latency benchmark workflow.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
green() { printf '\033[92m%s\033[0m\n' "$*"; }
yellow() { printf '\033[93m%s\033[0m\n' "$*"; }
red() { printf '\033[91m%s\033[0m\n' "$*"; }
cyan() { printf '\033[96m%s\033[0m\n' "$*"; }

REPEATS="${EMG_LATENCY_REPEATS:-5}"
BASELINE_S="${EMG_LATENCY_BASELINE_S:-1.5}"
TAIL_S="${EMG_LATENCY_TAIL_S:-0.5}"
COLLECT_REPS="${EMG_COLLECT_REPS:-3}"
COLLECT_DURATION="${EMG_COLLECT_DURATION_S:-8}"
MODEL_DIR="${EMG_MODEL_DIR:-/app/models}"
DATA_DIR="${EMG_DATA_DIR:-/app/data}"
RESULT_DIR_NAME="${EMG_LATENCY_RESULT_DIR_NAME:-latest}"
FETCH_REMOTE="${EMG_FETCH_REMOTE:-true}"
PUSH_RESULTS="${EMG_PUSH_RESULTS:-false}"
CONTAINER_NAME="prosthesis"
RESULT_DIR_HOST="${ROOT_DIR}/data/latency/${RESULT_DIR_NAME}"

if command -v podman-compose >/dev/null 2>&1; then
    COMPOSE_CMD=(podman-compose)
elif command -v docker >/dev/null 2>&1; then
    COMPOSE_CMD=(docker compose)
else
    red "Neither podman-compose nor docker is available."
    exit 1
fi

require_clean_branch() {
    local branch
    branch="$(git -C "$ROOT_DIR" branch --show-current)"
    if [ "$branch" != "asger_dev" ]; then
        red "This workflow only runs from the asger_dev branch. Current branch: $branch"
        exit 1
    fi
}

fetch_branch() {
    if [ "$FETCH_REMOTE" != "true" ]; then
        return
    fi
    cyan "Fetching origin/asger_dev with gh-authenticated git..."
    git -C "$ROOT_DIR" fetch origin asger_dev
}

ensure_container() {
    cyan "Starting prosthesis dev container if needed..."
    make -C "$ROOT_DIR" dev
}

container_exec() {
    (
        cd "$ROOT_DIR/docker"
        eval "${COMPOSE_CMD[*]} exec --user prosthesis -it ${CONTAINER_NAME} /bin/bash -lc \"$1\""
    )
}

run_collection() {
    bold "Step 1/4: EMG data collection"
    container_exec "source /opt/ros/jazzy/setup.bash && cd /prosthesis_ws && colcon build --packages-select emg_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release && source /prosthesis_ws/install/setup.bash && ros2 run emg_bridge collect_data --reps ${COLLECT_REPS} --duration ${COLLECT_DURATION} --output-dir ${DATA_DIR}"
}

run_training() {
    bold "Step 2/4: EMG model training"
    container_exec "source /opt/ros/jazzy/setup.bash && cd /prosthesis_ws && colcon build --packages-select emg_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release && source /prosthesis_ws/install/setup.bash && ros2 run emg_bridge train --data-dir ${DATA_DIR} --model-dir ${MODEL_DIR}"
}

run_latency_benchmark() {
    bold "Step 3/4: Interactive latency benchmark"
    mkdir -p "$RESULT_DIR_HOST"
    container_exec "source /opt/ros/jazzy/setup.bash && cd /prosthesis_ws && colcon build --packages-select emg_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release && source /prosthesis_ws/install/setup.bash && mkdir -p /app/data/latency/${RESULT_DIR_NAME} && ros2 run emg_bridge latency_benchmark --model-dir ${MODEL_DIR} --output-dir /app/data/latency/${RESULT_DIR_NAME} --repeats ${REPEATS} --baseline-s ${BASELINE_S} --tail-s ${TAIL_S}"
}

commit_and_push_results() {
    if [ "$PUSH_RESULTS" != "true" ]; then
        yellow "Skipping git commit/push of generated EMG artifacts. Set EMG_PUSH_RESULTS=true to enable it."
        return
    fi

    bold "Step 4/4: Commit and push generated EMG artifacts on asger_dev"
    git -C "$ROOT_DIR" add -f data models
    if git -C "$ROOT_DIR" diff --cached --quiet; then
        yellow "No generated artifact changes to commit."
        return
    fi
    git -C "$ROOT_DIR" commit -m "data: update EMG latency benchmark artifacts"
    local gh_token
    gh_token="$(gh auth token)"
    git -C "$ROOT_DIR" -c http.extraheader="AUTHORIZATION: bearer ${gh_token}" push origin HEAD:asger_dev
    green "Pushed asger_dev with generated EMG artifacts."
}

main() {
    require_clean_branch
    fetch_branch
    ensure_container
    run_collection
    run_training
    run_latency_benchmark
    commit_and_push_results

    green "Workflow complete. Results saved under data/latency/${RESULT_DIR_NAME}"
}

main "$@"
