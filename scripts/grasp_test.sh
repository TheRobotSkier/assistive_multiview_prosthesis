#!/usr/bin/env bash
set -euo pipefail

# ── Configuration ────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$PROJECT_ROOT/docker/docker-compose.yml"
COMPOSE_PROFILE="grasp_test"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ── Functions ────────────────────────────────────────────────────

check_prerequisites() {
    info "Checking prerequisites..."
    command -v podman-compose >/dev/null 2>&1 || { error "podman-compose not found"; return 1; }
    command -v ros2 >/dev/null 2>&1 || { warn "ros2 not found — will use docker exec"; }
    info "All prerequisites satisfied."
}

start_pipeline() {
    info "Starting grasp test pipeline..."
    cd "$PROJECT_ROOT"
    if [ "${USE_CAMERA:-false}" = "true" ]; then
        podman-compose --profile "$COMPOSE_PROFILE" up -d grasp_test
    else
        podman-compose --profile "$COMPOSE_PROFILE" up -d grasp_test
    fi
    info "Pipeline started. Waiting for nodes to initialize..."
    sleep 5
}

check_node_ready() {
    local node_name="$1"
    local container="grasp_test"
    if docker exec "$container" ros2 node list 2>/dev/null | grep -q "$node_name"; then
        return 0
    fi
    return 1
}

wait_for_node() {
    local node_name="$1"
    local timeout="${2:-30}"
    info "Waiting for $node_name (timeout: ${timeout}s)..."
    for i in $(seq 1 "$timeout"); do
        if check_node_ready "$node_name"; then
            info "$node_name is ready."
            return 0
        fi
        sleep 1
    done
    warn "$node_name not ready after ${timeout}s — continuing anyway"
    return 1
}

monitor_state() {
    local container="grasp_test"
    info "Monitoring pipeline states (Ctrl+C to stop)..."
    echo "─────────────────────────────────────"
    echo " Time    │ State"
    echo "─────────────────────────────────────"
    while true; do
        state=$(ros2 topic echo /pipeline/state_name --once --field data 2>/dev/null || echo "N/A")
        echo " $(date +%H:%M:%S) │ $state"
        sleep 1
    done
}

run_test_sequence() {
    info "Running automated test sequence..."
    # Start trajectory publisher if available
    ros2 param set /hand_trajectory_publisher auto_start true 2>/dev/null || {
        warn "hand_trajectory_publisher not available — manual movement required"
    }
    
    info "Waiting for grasp cycle to complete..."
    # Monitor until HOLDING state or timeout
    for i in $(seq 1 60); do
        state=$(ros2 topic echo /pipeline/state_name --once --field data 2>/dev/null || echo "")
        case "$state" in
            HOLDING)
                info "Grasp cycle complete! State: HOLDING"
                return 0
                ;;
            IDLE)
                if [ "$i" -gt 10 ]; then
                    warn "Returned to IDLE — grasp may have failed"
                fi
                ;;
        esac
        sleep 1
    done
    warn "Test timed out — expected HOLDING state not reached"
    return 1
}

stop_pipeline() {
    info "Stopping grasp test pipeline..."
    cd "$PROJECT_ROOT"
    podman-compose --profile "$COMPOSE_PROFILE" down
    info "Pipeline stopped."
}

# ── Main ─────────────────────────────────────────────────────────

main() {
    echo "═══════════════════════════════════════════════"
    echo "  Grasp Test Pipeline — Integration Test"
    echo "═══════════════════════════════════════════════"
    echo ""
    
    check_prerequisites || exit 1
    
    # Parse arguments
    USE_CAMERA=false
    MONITOR_ONLY=false
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --camera) USE_CAMERA=true ;;
            --monitor) MONITOR_ONLY=true ;;
            --help)
                echo "Usage: $0 [--camera] [--monitor]"
                exit 0
                ;;
            *) error "Unknown option: $1"; exit 1 ;;
        esac
        shift
    done
    
    if [ "$MONITOR_ONLY" = true ]; then
        monitor_state
        exit 0
    fi
    
    start_pipeline
    
    wait_for_node "pipeline_manager" 30
    wait_for_node "twist_propagation" 30
    wait_for_node "preshaping_service" 30
    wait_for_node "proximity_controller" 30
    
    run_test_sequence
    TEST_RESULT=$?
    
    stop_pipeline
    
    if [ $TEST_RESULT -eq 0 ]; then
        info "Test PASSED — successful grasp cycle"
        exit 0
    else
        warn "Test FAILED — grasp cycle incomplete"
        exit 1
    fi
}

main "$@"
