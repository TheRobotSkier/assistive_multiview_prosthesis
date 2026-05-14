#!/usr/bin/env bash
# scripts/jetson_run.sh — Run a command with guaranteed Jetson container cleanup
#
# Usage:  jetson_run.sh <command> [args...]
#
# Runs the given command, then unconditionally SSHes to the Jetson and kills
# all running Docker containers. Cleanup fires on success, failure, and on
# SIGTERM (i.e. when the agent-task-queue timeout expires).
#
# Agents MUST use this wrapper for all Jetson make/ssh jobs:
#
#   run_task(
#       command="scripts/jetson_run.sh make jetson-cameras",
#       working_directory="/home/asger/Drive/AAU/P8/grasping/multiview_prosthesis",
#       queue_name="jetson",
#       timeout_seconds=300
#   )

set -euo pipefail

JETSON_HOST="robotlab@robotlab.local"

_jetson_cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM  # prevent re-entrant cleanup on recursive signals

    echo "" >&2
    echo "=== [jetson_run] Stopping all Jetson containers ===" >&2
    ssh -o BatchMode=yes -o ConnectTimeout=10 \
        "$JETSON_HOST" \
        'running=$(docker ps -q 2>/dev/null)
         if [ -n "$running" ]; then
             echo "$running" | xargs docker kill && echo "All containers killed."
         else
             echo "No running containers."
         fi' \
        || echo "[jetson_run] Warning: cleanup SSH failed (Jetson unreachable?)" >&2

    exit "$exit_code"
}

trap _jetson_cleanup EXIT INT TERM

echo "=== [jetson_run] Running: $* ===" >&2
"$@"
