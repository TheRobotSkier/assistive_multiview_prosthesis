#!/usr/bin/env bash
# auto_iterate.sh — automated debugging loop for the prosthesis pipeline.
#
# Repeatedly:
#   1. Runs forge (non-interactive) to analyze the latest replay logs + iteration
#      history, decide what to fix, make the edits, and write an iteration log.
#   2. Runs the fast unit tests. If they fail, asks forge to fix the errors
#      (up to MAX_FIX_RETRIES times).
#   3. Runs the full replay regression test (make test-replay).
#   4. Repeats for N_ITERATIONS steps.
#
# The agent does NOT run the tests itself — the script does, after the agent
# finishes editing. This keeps the agent focused on code changes.
#
# Usage:
#   ./scripts/auto_iterate.sh                 # 30 iterations (default)
#   ./scripts/auto_iterate.sh 10              # 10 iterations
#   ITERATIONS=5 ./scripts/auto_iterate.sh    # via env var
#   MAX_FIX_RETRIES=3 ./scripts/auto_iterate.sh
#   RESUME_STEP=5 ./scripts/auto_iterate.sh   # resume from step 5
#   PHASE_TIMEOUT=600 ./scripts/auto_iterate.sh  # 10 min timeout per phase
#
# Prerequisites:
#   - prosthesis container running (make dev) and workspace built (make build)
#   - a replay baseline captured (make test-replay-baseline)
#   - forge CLI installed and authenticated
set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

ITERATIONS="${1:-${ITERATIONS:-30}}"
MAX_FIX_RETRIES="${MAX_FIX_RETRIES:-3}"
RESUME_STEP="${RESUME_STEP:-1}"
PHASE_TIMEOUT="${PHASE_TIMEOUT:-1200}"   # 20 min per phase
ITER_LOG_DIR="$REPO_ROOT/iteration-logs"
FORGE_LOG_DIR="$REPO_ROOT/logs/auto-iterate"

mkdir -p "$ITER_LOG_DIR" "$FORGE_LOG_DIR"

# Colors
if [ -t 1 ]; then
    B='\033[1m'; R='\033[31m'; G='\033[32m'; Y='\033[33m'; C='\033[36m'; D='\033[2m'; N='\033[0m'
else
    B=''; R=''; G=''; Y=''; C=''; D=''; N=''
fi

log()     { printf "${C}[%s]${N} %b\n" "$(date +%H:%M:%S)" "$*"; }
log_ok()   { printf "${G}[%s] ✓ %b${N}\n" "$(date +%H:%M:%S)" "$*"; }
log_warn() { printf "${Y}[%s] ! %b${N}\n" "$(date +%H:%M:%S)" "$*"; }
log_err()  { printf "${R}[%s] ✗ %b${N}\n" "$(date +%H:%M:%S)" "$*"; }
banner()   { printf "\n${B}${C}═══════════════════════════════════════════════════════════════%b${N}\n" "$*"; }

# ---------------------------------------------------------------------------
# Timeout wrapper — runs a command with a deadline.  If the command exceeds
# PHASE_TIMEOUT seconds it is killed and the wrapper returns non-zero.
# Usage: run_with_timeout <label> <command...>
# ---------------------------------------------------------------------------
run_with_timeout() {
    local label="$1"; shift
    local cmd=("$@")

    log "Running ${B}${label}${N} (timeout: ${PHASE_TIMEOUT}s)..."
    "${cmd[@]}" &
    local pid=$!

    local deadline=$(( PHASE_TIMEOUT + $(date +%s) ))
    local rc=0

    # Busy-wait with 1 s poll so we can kill quickly once time runs out
    while true; do
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null
            rc=$?
            break
        fi
        if [ "$(date +%s)" -ge "$deadline" ]; then
            log_warn "${label} timed out after ${PHASE_TIMEOUT}s — killing"
            kill -TERM "$pid" 2>/dev/null
            sleep 2
            if kill -0 "$pid" 2>/dev/null; then
                kill -KILL "$pid" 2>/dev/null
                wait "$pid" 2>/dev/null
            fi
            rc=124  # convention: timeout exit code
            break
        fi
        sleep 1
    done

    return $rc
}

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------

preflight() {
    banner "AUTO-ITERATE: pre-flight checks"

    # forge
    if ! command -v forge &>/dev/null; then
        log_err "forge CLI not found. Install it first."
        exit 1
    fi
    log_ok "forge found: $(forge --version 2>&1 | head -1)"

    # Container running
    local backend="${DOCKER_CMD:-podman}"
    if ! "$backend" inspect -f '{{.State.Running}}' prosthesis 2>/dev/null | grep -q true; then
        log_err "prosthesis container is not running."
        log_warn "Start it with: ${B}make dev${N} then ${B}make build${N}"
        exit 1
    fi
    log_ok "container 'prosthesis' is running"

    # Workspace built
    if ! "$backend" exec --user prosthesis prosthesis \
            /bin/bash -c 'test -f /prosthesis_ws/install/setup.bash' 2>/dev/null; then
        log_err "workspace not built (install/setup.bash missing)."
        log_warn "Run ${B}make build${N} inside the container first."
        exit 1
    fi
    log_ok "workspace is built"

    # Replay baseline exists
    if [ ! -f "$REPO_ROOT/tests/baselines/replay.json" ]; then
        log_warn "No replay baseline found at tests/baselines/replay.json"
        log_warn "The comparison will show 'no baseline' on the first replay run."
        log_warn "Run ${B}make test-replay-baseline${N} first for meaningful comparisons."
    else
        log_ok "replay baseline present"
    fi

    # iteration-logs dir
    if [ ! -d "$ITER_LOG_DIR" ]; then
        mkdir -p "$ITER_LOG_DIR"
        log "Created $ITER_LOG_DIR/"
    fi
    printf "\n"
}

# ---------------------------------------------------------------------------
# Phase 1: Run the forge agent (non-interactive) to make edits
# ---------------------------------------------------------------------------

run_agent() {
    local step="$1"
    local extra_context="${2:-}"
    local log_file="$FORGE_LOG_DIR/forge_step_${step}.log"

    banner "STEP ${step}/${ITERATIONS}: forge agent — analyze & fix"

    # Build the prompt. The agent reads iteration-logs/ + latest replay results,
    # decides what to work on, makes edits, and writes an iteration log entry.
    # It must NOT run tests — the script does that after.
    local prompt
    prompt="You are running from an automated debugging script (auto_iterate.sh), iteration ${step} of ${ITERATIONS}.

GOAL: A pipeline where:
  1. GTSAM factor-graph optimization clearly improves localization precision over raw VIO (visible in lower trajectory RMS, less drift, better odom→gtsam agreement).
  2. TSDF fusion produces a measurably better fused pointcloud than naivly concatenating clouds based on localization alone (visible in higher object cloud density, fewer outliers, tighter spatial consistency).

You are free to run investigative commands (e.g. grep, find, python3 -c ..., ros2 topic echo) inside or outside the container to understand the codebase, trace data flow, or verify assumptions.  If you create any temporary scripts or files, DELETE them before you finish so the workspace stays clean.

YOUR TASK:
1. Read the iteration-logs/ folder to understand what has been tried in previous iterations and what the current state is. If this is the first iteration, read the codebase structure to orient yourself.
2. Read the latest replay test results. These are in logs/replay-results/ — look at the most recent replay_<timestamp>/ directory. Key files: bag_analysis.txt, log_analysis.txt, bag_metrics.json, log_metrics.json. Also check logs/test-results/ for unit test results.
3. Based on what you find, decide on ONE main thing to investigate and try to solve this iteration. Prioritize: (a) any failing tests or errors, (b) metrics that regressed vs the baseline, (c) topics that are STARVED or have low effective_hz, (d) any known-failure patterns in the logs, (e) code paths where GTSAM smoothing or TSDF integration could be measurably tightened.
4. Make the code changes. Keep changes focused and minimal.
5. Write a new file in iteration-logs/ named iteration_${step}.md summarizing: what you saw in the logs, what you decided to work on, what changes you made (with file:line references), and what you expect the impact to be — specifically how it advances the goal of better localization precision or better fused pointclouds.

CONSTRAINTS:
- Do NOT run any tests yourself (make test, make test-replay, etc.). The script will run them after you finish.
- Do NOT start or stop the pipeline, container, or any ros2 processes.
- Keep changes focused — one problem per iteration.
- Always reference code changes as filepath:line.
- Clean up any temporary files or scripts you create before stopping.

${extra_context}

When you are done writing the iteration log file, you are finished. Stop."

    log "Running forge (non-interactive)... logs → ${D}${log_file}${N}"
    log "This may take several minutes."

    # Run forge non-interactively. Capture all output to the log file.
    # We pipe the prompt via -p for a single-shot session.
    if run_with_timeout "forge" forge -p "$prompt" >"$log_file" 2>&1; then
        log_ok "forge completed"
    else
        local rc=$?
        if [ "$rc" -eq 124 ]; then
            log_err "forge TIMED OUT after ${PHASE_TIMEOUT}s"
        else
            log_warn "forge exited with code ${rc} (may still have made edits)"
        fi
    fi

    # Check that an iteration log was written
    local iter_file="$ITER_LOG_DIR/iteration_${step}.md"
    if [ -f "$iter_file" ]; then
        log_ok "iteration log written: ${D}iteration-logs/iteration_${step}.md${N}"
    else
        log_warn "no iteration log found at iteration_${step}.md — agent may not have completed fully"
    fi
    printf "\n"
}

# ---------------------------------------------------------------------------
# Phase 2: Run fast unit tests, with fix-on-failure retry loop
# ---------------------------------------------------------------------------

run_unit_tests() {
    local step="$1"
    local attempt=0

    while true; do
        attempt=$((attempt + 1))
        banner "STEP ${step}: unit tests (attempt ${attempt})"

        log "Running ${B}make test-unit${N} ..."
        local backend="${DOCKER_CMD:-podman}"

        # ---- Build Python packages before test ----
        # The develop install (--symlink-install) symlinks source -> build -> install
        # so Python source changes are reflected immediately.  Packaging changes
        # (setup.py, new scripts) need a build first.
        log "Building Python packages (gtsam_tracker, cross_camera_features, keyframe_buffer)..."
        "$backend" exec --user prosthesis prosthesis /bin/bash -lc \
            'source /opt/ros/jazzy/setup.bash && \
             cd /prosthesis_ws && \
             colcon build --packages-select gtsam_tracker cross_camera_features keyframe_buffer \
               --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1' \
            >>"$FORGE_LOG_DIR/unit_test_step_${step}_attempt_${attempt}.log" 2>&1 || \
            log_warn "Python package build had issues (non-fatal)"

        # test-unit runs inside the container via the workspace Makefile
        if run_with_timeout "unit-tests" \
                "$backend" exec --user prosthesis prosthesis /bin/bash -lc \
                'source /opt/ros/jazzy/setup.bash && \
                 source /prosthesis_ws/install/setup.bash 2>/dev/null && \
                 cd /prosthesis_ws && \
                 bash scripts/test_unit.sh' \
                >"$FORGE_LOG_DIR/unit_test_step_${step}_attempt_${attempt}.log" 2>&1; then
            log_ok "unit tests passed"
            return 0
        fi

        log_err "unit tests FAILED (attempt ${attempt}/${MAX_FIX_RETRIES})"

        if [ "$attempt" -ge "$MAX_FIX_RETRIES" ]; then
            log_warn "max retries (${MAX_FIX_RETRIES}) reached — proceeding to replay test anyway"
            return 1
        fi

        # Ask forge to fix the test failures
        log "Asking forge to fix the unit test failures..."
        local fail_log="$FORGE_LOG_DIR/unit_test_step_${step}_attempt_${attempt}.log"
        local fix_prompt
        fix_prompt="You are running from an automated debugging script, iteration ${step}, unit-test-fix attempt ${attempt}.

The unit tests just FAILED. Your task: fix the errors so the tests pass.

The test output is saved at ${fail_log} (relative to the repo root). Read it to see the failures.

Also read the iteration-logs/iteration_${step}.md you wrote earlier for context on what you changed.

Fix the failing tests. If a test is failing because of your recent changes, fix your code. If a test itself is broken, fix the test. Keep changes minimal.

Do NOT run any tests yourself. Do NOT write an iteration log. Just make the fix and stop."

        local fix_log="$FORGE_LOG_DIR/forge_fix_step_${step}_attempt_${attempt}.log"
        if run_with_timeout "forge-fix" forge -p "$fix_prompt" >"$fix_log" 2>&1; then
            log_ok "forge fix completed"
        else
            local frc=$?
            if [ "$frc" -eq 124 ]; then
                log_err "forge fix TIMED OUT after ${PHASE_TIMEOUT}s"
            else
                log_warn "forge fix exited non-zero — retrying tests anyway"
            fi
        fi
    done
}

# ---------------------------------------------------------------------------
# Phase 3: Run the full replay regression test
# ---------------------------------------------------------------------------

run_replay_test() {
    local step="$1"
    local log_file="$FORGE_LOG_DIR/replay_step_${step}.log"

    banner "STEP ${step}: replay regression test"

    log "Running ${B}make test-replay${N} ..."
    log "This takes ~3 minutes (166s bag replay + analysis)."

    if run_with_timeout "test-replay" make test-replay >"$log_file" 2>&1; then
        log_ok "replay test PASSED — no regressions"
    else
        local rc=$?
        if [ "$rc" -eq 124 ]; then
            log_err "replay test TIMED OUT after ${PHASE_TIMEOUT}s"
        else
            log_warn "replay test reported issues (exit ${rc})"
        fi
        log "See ${D}${log_file}${N} for the full comparison output."
    fi

    # Identify the latest replay run dir for the next iteration's agent to read
    local latest
    latest="$(ls -td "$REPO_ROOT"/logs/replay-results/replay_* 2>/dev/null | head -1)"
    if [ -n "$latest" ]; then
        log "Latest replay results: ${D}$(basename "$latest")${N}"
    fi
    printf "\n"
}

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

print_summary() {
    banner "AUTO-ITERATE COMPLETE"

    log "Iterations run: ${ITERATIONS}"
    log "Iteration logs: ${D}${ITER_LOG_DIR#$REPO_ROOT/}/${N}"
    log "Forge logs:     ${D}${FORGE_LOG_DIR#$REPO_ROOT/}/${N}"
    echo ""
    log "Iteration log files written:"
    ls -1 "$ITER_LOG_DIR"/iteration_*.md 2>/dev/null | sort -V | while read -r f; do
        printf "  ${D}%s${N}\n" "${f#$REPO_ROOT/}"
    done
    echo ""
    log "Replay results:"
    ls -1d "$REPO_ROOT"/logs/replay-results/replay_* 2>/dev/null | sort | while read -r d; do
        printf "  ${D}%s${N}\n" "${d#$REPO_ROOT/}"
    done
}

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

main() {
    preflight

    banner "STARTING AUTO-ITERATE: ${ITERATIONS} iterations"
    log "Resume step:  ${RESUME_STEP}"
    log "Phase timeout: ${PHASE_TIMEOUT}s per phase"
    log "Fast tests (unit) gate each step; replay test runs after."
    log "Forge logs: ${D}${FORGE_LOG_DIR#$REPO_ROOT/}/${N}"
    log "Iteration logs: ${D}${ITER_LOG_DIR#$REPO_ROOT/}/${N}"
    printf "\n"

    local start_time
    start_time=$(date +%s)

    for step in $(seq 1 "$ITERATIONS"); do
        # -------- Resume support --------
        if [ "$step" -lt "$RESUME_STEP" ]; then
            log "Skipping step ${step} (resuming from ${RESUME_STEP})"
            continue
        fi

        local step_start
        step_start=$(date +%s)

        # Phase 1: agent analyzes + edits + writes iteration log
        run_agent "$step"

        # Phase 2: fast unit tests (with fix retry loop)
        run_unit_tests "$step"

        # Phase 3: full replay regression test
        run_replay_test "$step"

        local elapsed=$(( $(date +%s) - step_start ))
        local total=$(( $(date +%s) - start_time ))
        log "Step ${step} done in ${elapsed}s (total elapsed: ${total}s)"
        printf "\n"
    done

    print_summary
}

main "$@"
