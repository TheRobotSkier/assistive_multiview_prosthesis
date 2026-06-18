#!/bin/bash
# EMG collection + training + latency benchmark workflow.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_PATH="${EMG_LATENCY_CONFIG:-${ROOT_DIR}/config/emg_latency_test.yaml}"

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
green() { printf '\033[92m%s\033[0m\n' "$*"; }
yellow() { printf '\033[93m%s\033[0m\n' "$*"; }
red() { printf '\033[91m%s\033[0m\n' "$*"; }
cyan() { printf '\033[96m%s\033[0m\n' "$*"; }

yaml_value() {
    local path="$1"
    local default="$2"
    python3 - "$CONFIG_PATH" "$path" "$default" <<'PY'
import sys
try:
    import yaml
except Exception:
    print(sys.argv[3])
    sys.exit(0)

cfg_path, dotted, default = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    with open(cfg_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
except FileNotFoundError:
    print(default)
    sys.exit(0)

cur = data
for part in dotted.split("."):
    if not isinstance(cur, dict) or part not in cur:
        print(default)
        sys.exit(0)
    cur = cur[part]

if isinstance(cur, bool):
    print("true" if cur else "false")
else:
    print(cur)
PY
}

COLLECT_REPS_DEFAULT="$(yaml_value workflow.collect_reps 3)"
COLLECT_DURATION_DEFAULT="$(yaml_value workflow.collect_duration_s 8)"
MODEL_DIR_DEFAULT="$(yaml_value workflow.model_dir /app/models)"
DATA_DIR_DEFAULT="$(yaml_value workflow.data_dir /app/data)"
RESULT_DIR_NAME_DEFAULT="$(yaml_value workflow.result_dir_name auto)"
FETCH_REMOTE_DEFAULT="$(yaml_value workflow.fetch_remote false)"
PUSH_RESULTS_DEFAULT="$(yaml_value workflow.push_results false)"
SKIP_TRAIN_DEFAULT="$(yaml_value workflow.skip_train false)"
REPEATS_DEFAULT="$(yaml_value benchmark.repeats 5)"
BASELINE_S_DEFAULT="$(yaml_value benchmark.baseline_s 1.5)"
TAIL_S_DEFAULT="$(yaml_value benchmark.tail_s 0.5)"
THRESHOLD_DEFAULT="$(yaml_value benchmark.confidence_threshold 0.55)"
SMOOTH_DEFAULT="$(yaml_value benchmark.smoothing_frames 5)"
COUNTDOWN_S_DEFAULT="$(yaml_value benchmark.countdown_s 3)"
AUTO_STOP_CONFIDENCE_DEFAULT="$(yaml_value benchmark.auto_stop_confidence 0.70)"
AUTO_STOP_HOLD_S_DEFAULT="$(yaml_value benchmark.auto_stop_hold_s 1.0)"
MAX_TRIAL_DURATION_S_DEFAULT="$(yaml_value benchmark.max_trial_duration_s 15.0)"
MIN_ACTIVE_SAMPLES_DEFAULT="$(yaml_value benchmark.min_active_samples 6)"
MAX_GAP_SAMPLES_DEFAULT="$(yaml_value benchmark.max_gap_samples 2)"
PRE_ONSET_SEARCH_SAMPLES_DEFAULT="$(yaml_value benchmark.pre_onset_search_samples 20)"
ONSET_LOOKBACK_WINDOWS_DEFAULT="$(yaml_value benchmark.onset_lookback_windows 5)"

if [ "$RESULT_DIR_NAME_DEFAULT" = "auto" ] || [ -z "$RESULT_DIR_NAME_DEFAULT" ]; then
    RESULT_DIR_NAME_DEFAULT="$(date +%Y%m%d_%H%M%S)"
fi

REPEATS="${EMG_LATENCY_REPEATS:-$REPEATS_DEFAULT}"
BASELINE_S="${EMG_LATENCY_BASELINE_S:-$BASELINE_S_DEFAULT}"
TAIL_S="${EMG_LATENCY_TAIL_S:-$TAIL_S_DEFAULT}"
THRESHOLD="${EMG_LATENCY_THRESHOLD:-$THRESHOLD_DEFAULT}"
SMOOTH="${EMG_LATENCY_SMOOTH:-$SMOOTH_DEFAULT}"
COUNTDOWN_S="${EMG_LATENCY_COUNTDOWN_S:-$COUNTDOWN_S_DEFAULT}"
AUTO_STOP_CONFIDENCE="${EMG_LATENCY_AUTO_STOP_CONFIDENCE:-$AUTO_STOP_CONFIDENCE_DEFAULT}"
AUTO_STOP_HOLD_S="${EMG_LATENCY_AUTO_STOP_HOLD_S:-$AUTO_STOP_HOLD_S_DEFAULT}"
MAX_TRIAL_DURATION_S="${EMG_LATENCY_MAX_TRIAL_DURATION_S:-$MAX_TRIAL_DURATION_S_DEFAULT}"
MIN_ACTIVE_SAMPLES="${EMG_LATENCY_MIN_ACTIVE_SAMPLES:-$MIN_ACTIVE_SAMPLES_DEFAULT}"
MAX_GAP_SAMPLES="${EMG_LATENCY_MAX_GAP_SAMPLES:-$MAX_GAP_SAMPLES_DEFAULT}"
PRE_ONSET_SEARCH_SAMPLES="${EMG_LATENCY_PRE_ONSET_SEARCH_SAMPLES:-$PRE_ONSET_SEARCH_SAMPLES_DEFAULT}"
ONSET_LOOKBACK_WINDOWS="${EMG_LATENCY_ONSET_LOOKBACK_WINDOWS:-$ONSET_LOOKBACK_WINDOWS_DEFAULT}"
COLLECT_REPS="${EMG_COLLECT_REPS:-$COLLECT_REPS_DEFAULT}"
COLLECT_DURATION="${EMG_COLLECT_DURATION_S:-$COLLECT_DURATION_DEFAULT}"
MODEL_DIR="${EMG_MODEL_DIR:-$MODEL_DIR_DEFAULT}"
DATA_DIR="${EMG_DATA_DIR:-$DATA_DIR_DEFAULT}"
RESULT_DIR_NAME="${EMG_LATENCY_RESULT_DIR_NAME:-$RESULT_DIR_NAME_DEFAULT}"
FETCH_REMOTE="${EMG_FETCH_REMOTE:-$FETCH_REMOTE_DEFAULT}"
PUSH_RESULTS="${EMG_PUSH_RESULTS:-$PUSH_RESULTS_DEFAULT}"
EMG_SKIP_TRAIN="${EMG_SKIP_TRAIN:-$SKIP_TRAIN_DEFAULT}"
CONTAINER_NAME="emg"
RESULT_DIR_HOST="${ROOT_DIR}/data/latency/${RESULT_DIR_NAME}"

if command -v podman-compose >/dev/null 2>&1; then
    COMPOSE_CMD=(podman-compose)
    DOCKER_BIN="podman"
elif command -v docker >/dev/null 2>&1; then
    COMPOSE_CMD=(docker compose)
    DOCKER_BIN="docker"
else
    red "Neither podman-compose nor docker is available."
    exit 1
fi

cleanup_container() {
    "$DOCKER_BIN" stop --time 1 "$CONTAINER_NAME" 2>/dev/null || true
    "$DOCKER_BIN" rm -f "$CONTAINER_NAME" 2>/dev/null || true
}

trap cleanup_container EXIT

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
    cyan "Fetching origin/asger_dev because EMG_FETCH_REMOTE=true..."
    git -C "$ROOT_DIR" fetch origin asger_dev
}

ensure_container() {
    cleanup_container
    cyan "Starting dedicated EMG container if needed..."
    make -C "$ROOT_DIR" emg-dev
}

container_exec() {
    (
        cd "$ROOT_DIR/docker"
        eval "${COMPOSE_CMD[*]} exec --user prosthesis ${CONTAINER_NAME} /bin/bash -lc \"$1\""
    )
}

run_collection() {
    if [ "$EMG_SKIP_TRAIN" = "true" ]; then
        yellow "Skipping EMG data collection and reusing existing files under ${DATA_DIR} and ${MODEL_DIR}."
        return
    fi
    bold "Step 1/4: EMG data collection"
    container_exec "source /opt/ros/jazzy/setup.bash && cd /prosthesis_ws && colcon build --packages-select emg_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release && source /prosthesis_ws/install/setup.bash && ros2 run emg_bridge collect_data --reps ${COLLECT_REPS} --duration ${COLLECT_DURATION} --output-dir ${DATA_DIR} ${EMG_BOARD_IP:+--ip ${EMG_BOARD_IP}}"
}

run_training() {
    if [ "$EMG_SKIP_TRAIN" = "true" ]; then
        yellow "Skipping EMG model training and reusing files under ${MODEL_DIR}."
        return
    fi
    bold "Step 2/4: EMG model training"
    container_exec "source /opt/ros/jazzy/setup.bash && cd /prosthesis_ws && colcon build --packages-select emg_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release && source /prosthesis_ws/install/setup.bash && ros2 run emg_bridge train --data-dir ${DATA_DIR} --model-dir ${MODEL_DIR}"
}

run_latency_benchmark() {
    bold "Step 3/4: Interactive latency benchmark"
    mkdir -p "$RESULT_DIR_HOST"
    container_exec "source /opt/ros/jazzy/setup.bash && cd /prosthesis_ws && colcon build --packages-select emg_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release && source /prosthesis_ws/install/setup.bash && mkdir -p /app/data/latency/${RESULT_DIR_NAME} && ros2 run emg_bridge latency_benchmark --model-dir ${MODEL_DIR} --output-dir /app/data/latency/${RESULT_DIR_NAME} --repeats ${REPEATS} --baseline-s ${BASELINE_S} --tail-s ${TAIL_S} --threshold ${THRESHOLD} --smooth ${SMOOTH} --countdown-s ${COUNTDOWN_S} --auto-stop-confidence ${AUTO_STOP_CONFIDENCE} --auto-stop-hold-s ${AUTO_STOP_HOLD_S} --max-trial-duration-s ${MAX_TRIAL_DURATION_S} --min-active-samples ${MIN_ACTIVE_SAMPLES} --max-gap-samples ${MAX_GAP_SAMPLES} --pre-onset-search-samples ${PRE_ONSET_SEARCH_S} --onset-lookback-windows ${ONSET_LOOKBACK_WINDOWS} ${EMG_BOARD_IP:+--ip ${EMG_BOARD_IP}}"
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
    bold "EMG latency test workflow"
    echo "Config YAML: $CONFIG_PATH"
    echo "Results: data/latency/${RESULT_DIR_NAME}"
    echo ""
    require_clean_branch
    fetch_branch
    ensure_container
    run_collection
    run_training
    if [ "${EMG_SKIP_BENCHMARK:-false}" != "true" ]; then
        run_latency_benchmark
    else
        green "Skipping benchmark (EMG_SKIP_BENCHMARK=true)."
    fi
    commit_and_push_results

    green "Workflow complete. Results saved under data/latency/${RESULT_DIR_NAME}"
}

main "$@"
