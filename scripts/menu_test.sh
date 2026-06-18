#!/bin/bash
# menu_test.sh — Interactive test selector for the Prosthesis project.
#
# Usage: bash scripts/menu_test.sh
# On space: writes selected test to /tmp/prosthesis_test_choice and exits 0
# On q:     exits 1
# Press 'e' on a test to edit its env parameters.
set -e

# ── Test definitions ─────────────────────────────────────────────────
TESTS_LABEL=(
    "Haptic Force Test"
    "EMG Force Grasp"
    "Collect + Train EMG"
    "EMG Latency Benchmark"
    "Static Grasp Test"
    "Reset Hand + Haptics"
)
TESTS_TARGET=(
    "test-grasp"
    "emg-force-grasp"
    "emg-collect-train"
    "test-emg-latency"
    "test-static-grasp"
    "test-reset"
)
TESTS_PARAMS=(
    "yes"
    "yes"
    "no"
    "yes"
    "no"
    "no"
)
NUM_TESTS=${#TESTS_LABEL[@]}
# ── ANSI helpers ─────────────────────────────────────────────────────
BOLD="$(printf '\e[1m')"; DIM="$(printf '\e[2m')"; REV="$(printf '\e[7m')"
GREEN="$(printf '\e[32m')"; CYAN="$(printf '\e[36m')"; YELLOW="$(printf '\e[33m')"
RESET="$(printf '\e[0m')"
hide_cursor() { printf '\e[?25l'; }
show_cursor() { printf '\e[?25h'; }
clear_screen() { printf '\e[H\e[J'; }

read_key() {
    local key
    IFS= read -rsn1 key 2>/dev/null || { echo "q"; return; }
    if [[ $key == $'\e' ]]; then
        IFS= read -rsn2 -t 0.001 key 2>/dev/null || true
        case "$key" in '[A') echo UP ;; '[B') echo DOWN ;; *) echo ESC ;; esac
    elif [[ $key == '' ]]; then
        echo ENTER
    else
        echo "$key"
    fi
}

_load_yaml_defaults() {
    # Baked defaults from config/mia_haptic_force_test.yaml.
    # These are used when pyyaml is unavailable on the host.
    AUTO_KILL_S="${AUTO_KILL_S:-0}"
    MOCK_HARDWARE="${MOCK_HARDWARE:-auto}"
    CONTROL_RATE_HZ="${CONTROL_RATE_HZ:-50.0}"
    CSV_RATE_HZ="${CSV_RATE_HZ:-10.0}"
    HAPTICS_RATE_HZ="${HAPTICS_RATE_HZ:-10.0}"
    TERMINAL_RATE_HZ="${TERMINAL_RATE_HZ:-2.0}"
    STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-30.0}"
    OPEN_THUMB="${OPEN_THUMB:-0.0}"; OPEN_INDEX="${OPEN_INDEX:-0.0}"; OPEN_MRL="${OPEN_MRL:-0.0}"
    MAXCLOSE_THUMB="${MAXCLOSE_THUMB:-1.5}"; MAXCLOSE_INDEX="${MAXCLOSE_INDEX:-1.5}"; MAXCLOSE_MRL="${MAXCLOSE_MRL:-1.5}"
    OPEN_TOLERANCE="${OPEN_TOLERANCE:-0.08}"; OPEN_MIN_S="${OPEN_MIN_S:-0.5}"
    OPEN_TIMEOUT_S="${OPEN_TIMEOUT_S:-4.0}"
    CLOSE_VEL_START="${CLOSE_VEL_START:-0.3}"; CLOSE_VEL_END="${CLOSE_VEL_END:-0.1}"
    CLOSE_DECAY="${CLOSE_DECAY:-5}"; CLOSE_INTERVAL="${CLOSE_INTERVAL:-0.2}"
    HOLD_DEADZONE="${HOLD_DEADZONE:-20}"; HOLD_MIN_OVER="${HOLD_MIN_OVER:-20}"
    HOLD_MAX_OVER="${HOLD_MAX_OVER:-150}"; HOLD_MAX_VEL="${HOLD_MAX_VEL:-0.08}"
    F_THUMB="${F_THUMB:-300}"; F_INDEX="${F_INDEX:-300}"; F_MRL="${F_MRL:-300}"
    F_INIT_THUMB="${F_INIT_THUMB:-300}"; F_INIT_INDEX="${F_INIT_INDEX:-300}"; F_INIT_MRL="${F_INIT_MRL:-300}"
    F_TARGET_MIN="${F_TARGET_MIN:-50}"; F_TARGET_MAX="${F_TARGET_MAX:-500}"
    F_ADJ_UP="${F_ADJ_UP:-100}"; F_ADJ_DOWN="${F_ADJ_DOWN:-200}"
    F_EMERGENCY="${F_EMERGENCY:-800}"; F_BACKOFF="${F_BACKOFF:--0.1}"
    EMG_CONF="${EMG_CONF:-0.55}"; EMG_ACTIVATION_HOLD="${EMG_ACTIVATION_HOLD:-0.2}"
    EMG_OPEN_HOLD="${EMG_OPEN_HOLD:-1.0}"; EMG_TOGGLE_HOLD="${EMG_TOGGLE_HOLD:-0.5}"
    EMG_STALE_TIMEOUT="${EMG_STALE_TIMEOUT:-1.0}"
    W_HORIZONTAL="${W_HORIZONTAL:-180}"; W_VERTICAL="${W_VERTICAL:-90}"
    W_MIN_DEG="${W_MIN_DEG:-5}"; W_MAX_DEG="${W_MAX_DEG:-300}"
    W_ACCEL="${W_ACCEL:-180}"; W_CTRL_VEL="${W_CTRL_VEL:-45}"
    W_TOLERANCE="${W_TOLERANCE:-4}"; W_TIMEOUT="${W_TIMEOUT:-8}"
    W_RETURN_DELAY="${W_RETURN_DELAY:-3}"; W_VERT_DELAY="${W_VERT_DELAY:-3}"

    EMG_BOARD_IP="${EMG_BOARD_IP:-10.27.30.3}"
    WRIST_BOARD_IP="${WRIST_BOARD_IP:-}"
    # The make test menu should launch the repaired split-node stack by default.
    # Set USE_MULTI_NODE=false explicitly to use the legacy monolithic fallback.
    USE_MULTI_NODE="${USE_MULTI_NODE:-true}"

    local yaml="${CONFIG_PATH:-config/mia_haptic_force_test.yaml}"
    [ -f "$yaml" ] || return 0

    local assignments
    assignments="$(python3 - "$yaml" 2>/dev/null <<'PY'
import shlex
import sys

import yaml

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}


def get(path, default=None):
    cur = data
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def emit(name, value):
    if value is None:
        return
    if isinstance(value, bool):
        value = "true" if value else "false"
    else:
        value = str(value)
    print(f"{name}={shlex.quote(value)}")


paths = {
    "AUTO_KILL_S": "runtime.auto_kill_s",
    "MOCK_HARDWARE": "runtime.mock_hardware",
    "USE_MULTI_NODE": "runtime.use_multi_node",
    "CONTROL_RATE_HZ": "runtime.control_rate_hz",
    "CSV_RATE_HZ": "runtime.csv_rate_hz",
    "HAPTICS_RATE_HZ": "runtime.haptics_publish_rate_hz",
    "TERMINAL_RATE_HZ": "runtime.terminal_rate_hz",
    "STARTUP_TIMEOUT_S": "runtime.startup_controller_timeout_s",
    "OPEN_THUMB": "hand.open_positions.j_thumb_fle",
    "OPEN_INDEX": "hand.open_positions.j_index_fle",
    "OPEN_MRL": "hand.open_positions.j_mrl_fle",
    "MAXCLOSE_THUMB": "hand.max_closure_positions.j_thumb_fle",
    "MAXCLOSE_INDEX": "hand.max_closure_positions.j_index_fle",
    "MAXCLOSE_MRL": "hand.max_closure_positions.j_mrl_fle",
    "OPEN_TOLERANCE": "hand.open_position_tolerance_rad",
    "OPEN_MIN_S": "hand.opening_min_s",
    "OPEN_TIMEOUT_S": "hand.opening_timeout_s",
    "CLOSE_VEL_START": "hand.closing_velocity_start_rad_s",
    "CLOSE_VEL_END": "hand.closing_velocity_end_rad_s",
    "CLOSE_DECAY": "hand.closing_decay_steps",
    "CLOSE_INTERVAL": "hand.closing_step_interval_s",
    "HOLD_DEADZONE": "hand.hold_deadzone",
    "HOLD_MIN_OVER": "hand.hold_min_overshoot",
    "HOLD_MAX_OVER": "hand.hold_max_overshoot",
    "HOLD_MAX_VEL": "hand.hold_max_velocity_rad_s",
    "F_THUMB": "force.contact_thresholds.j_thumb_fle",
    "F_INDEX": "force.contact_thresholds.j_index_fle",
    "F_MRL": "force.contact_thresholds.j_mrl_fle",
    "F_INIT_THUMB": "force.initial_hold_targets.j_thumb_fle",
    "F_INIT_INDEX": "force.initial_hold_targets.j_index_fle",
    "F_INIT_MRL": "force.initial_hold_targets.j_mrl_fle",
    "F_TARGET_MIN": "force.target_min",
    "F_TARGET_MAX": "force.target_max",
    "F_ADJ_UP": "force.adjustment_rate_up",
    "F_ADJ_DOWN": "force.adjustment_rate_down",
    "F_EMERGENCY": "force.emergency_threshold",
    "F_BACKOFF": "force.emergency_backoff_velocity_rad_s",
    "EMG_BOARD_IP": "emg.board_ip",
    "EMG_CONF": "emg.confidence_threshold",
    "EMG_ACTIVATION_HOLD": "emg.activation_hold_s",
    "EMG_OPEN_HOLD": "emg.open_hold_s",
    "EMG_TOGGLE_HOLD": "emg.power_toggle_hold_s",
    "EMG_STALE_TIMEOUT": "emg.stale_timeout_s",
    "W_HORIZONTAL": "wrist.horizontal_deg",
    "W_VERTICAL": "wrist.vertical_deg",
    "W_MIN_DEG": "wrist.min_deg",
    "W_MAX_DEG": "wrist.max_deg",
    "W_ACCEL": "wrist.acceleration_deg_s2",
    "W_CTRL_VEL": "wrist.control_velocity_deg_s",
    "W_TOLERANCE": "wrist.position_tolerance_deg",
    "W_TIMEOUT": "wrist.move_timeout_s",
    "W_RETURN_DELAY": "wrist.return_after_open_delay_s",
    "W_VERT_DELAY": "wrist.vertical_delay_s",
}

for name, yaml_path in paths.items():
    emit(name, get(yaml_path))
PY
)" || return 0
    eval "$assignments"
}
# ── Write env file ───────────────────────────────────────────────────
write_env_file() {
    local f="${1:-/tmp/prosthesis_test_env.sh}"
    {
        echo "#!/bin/bash"; echo "# Generated by menu_test.sh"
        for v in MIA_PORT WRIST_PORT MOCK_HARDWARE WRIST_ENABLE HAPTIC_ENABLE \
                 FORCE_RETRAIN AUTO_KILL_S CONFIG_PATH CONTROL_RATE_HZ CSV_RATE_HZ \
                 HAPTICS_RATE_HZ TERMINAL_RATE_HZ STARTUP_TIMEOUT_S EMG_BOARD_IP \
                 USE_MULTI_NODE \
                 OPEN_THUMB OPEN_INDEX OPEN_MRL MAXCLOSE_THUMB MAXCLOSE_INDEX MAXCLOSE_MRL \
                 OPEN_TOLERANCE OPEN_MIN_S OPEN_TIMEOUT_S CLOSE_VEL_START CLOSE_VEL_END \
                 CLOSE_DECAY CLOSE_INTERVAL HOLD_DEADZONE HOLD_MIN_OVER HOLD_MAX_OVER \
                 HOLD_MAX_VEL F_THUMB F_INDEX F_MRL F_INIT_THUMB F_INIT_INDEX F_INIT_MRL \
                 F_TARGET_MIN F_TARGET_MAX F_ADJ_UP F_ADJ_DOWN F_EMERGENCY F_BACKOFF \
                 W_HORIZONTAL W_VERTICAL W_MIN_DEG W_MAX_DEG W_ACCEL W_CTRL_VEL W_TOLERANCE \
                 W_TIMEOUT W_RETURN_DELAY W_VERT_DELAY; do
            echo "export $v='${!v}'"
        done
    } > "$f"
}

save_config() {
    local yaml="${CONFIG_PATH:-config/mia_haptic_force_test.yaml}"
    local tmp="${yaml}.tmp"
    if python3 -c "
import yaml, sys
d = yaml.safe_load(open('$yaml'))
# Apply overrides
d.setdefault('runtime',{})
d['runtime']['auto_kill_s'] = float('${AUTO_KILL_S:-0}')
d['runtime']['mock_hardware'] = '${MOCK_HARDWARE:-auto}'
d['runtime']['use_multi_node'] = str('${USE_MULTI_NODE:-true}').lower() in ('1','true','yes','on')
d['runtime']['control_rate_hz'] = float('${CONTROL_RATE_HZ:-50.0}')
d['runtime']['csv_rate_hz'] = float('${CSV_RATE_HZ:-10.0}')
d['runtime']['haptics_publish_rate_hz'] = float('${HAPTICS_RATE_HZ:-10.0}')
d['runtime']['terminal_rate_hz'] = float('${TERMINAL_RATE_HZ:-2.0}')
d['runtime']['startup_controller_timeout_s'] = float('${STARTUP_TIMEOUT_S:-30.0}')
d.setdefault('hand',{})
d['hand']['open_positions'] = {'j_thumb_fle':float('${OPEN_THUMB:-0.0}'),'j_index_fle':float('${OPEN_INDEX:-0.0}'),'j_mrl_fle':float('${OPEN_MRL:-0.0}')}
d['hand']['max_closure_positions'] = {'j_thumb_fle':float('${MAXCLOSE_THUMB:-1.5}'),'j_index_fle':float('${MAXCLOSE_INDEX:-1.5}'),'j_mrl_fle':float('${MAXCLOSE_MRL:-1.5}')}
d['hand']['open_position_tolerance_rad'] = float('${OPEN_TOLERANCE:-0.08}')
d['hand']['opening_min_s'] = float('${OPEN_MIN_S:-0.5}')
d['hand']['opening_timeout_s'] = float('${OPEN_TIMEOUT_S:-4.0}')
d['hand']['closing_velocity_start_rad_s'] = float('${CLOSE_VEL_START:-0.3}')
d['hand']['closing_velocity_end_rad_s'] = float('${CLOSE_VEL_END:-0.1}')
d['hand']['closing_decay_steps'] = int('${CLOSE_DECAY:-5}')
d['hand']['closing_step_interval_s'] = float('${CLOSE_INTERVAL:-0.2}')
d['hand']['hold_deadzone'] = float('${HOLD_DEADZONE:-20}')
d['hand']['hold_min_overshoot'] = float('${HOLD_MIN_OVER:-20}')
d['hand']['hold_max_overshoot'] = float('${HOLD_MAX_OVER:-150}')
d['hand']['hold_max_velocity_rad_s'] = float('${HOLD_MAX_VEL:-0.08}')
d.setdefault('force',{})
d['force']['contact_thresholds'] = {'j_thumb_fle':float('${F_THUMB:-300}'),'j_index_fle':float('${F_INDEX:-300}'),'j_mrl_fle':float('${F_MRL:-300}')}
d['force']['initial_hold_targets'] = {'j_thumb_fle':float('${F_INIT_THUMB:-300}'),'j_index_fle':float('${F_INIT_INDEX:-300}'),'j_mrl_fle':float('${F_INIT_MRL:-300}')}
d['force']['target_min'] = float('${F_TARGET_MIN:-50}')
d['force']['target_max'] = float('${F_TARGET_MAX:-500}')
d['force']['adjustment_rate_up'] = float('${F_ADJ_UP:-100}')
d['force']['adjustment_rate_down'] = float('${F_ADJ_DOWN:-200}')
d['force']['emergency_threshold'] = float('${F_EMERGENCY:-800}')
d['force']['emergency_backoff_velocity_rad_s'] = float('${F_BACKOFF:--0.1}')
d.setdefault('emg',{})
d['emg']['board_ip'] = '${EMG_BOARD_IP:-10.27.30.3}'
d['emg']['confidence_threshold'] = float('${EMG_CONF:-0.55}')
d['emg']['activation_hold_s'] = float('${EMG_ACTIVATION_HOLD:-0.2}')
d['emg']['open_hold_s'] = float('${EMG_OPEN_HOLD:-1.0}')
d['emg']['power_toggle_hold_s'] = float('${EMG_TOGGLE_HOLD:-0.5}')
d['emg']['stale_timeout_s'] = float('${EMG_STALE_TIMEOUT:-1.0}')
d.setdefault('wrist',{})
d['wrist']['horizontal_deg'] = float('${W_HORIZONTAL:-180}')
d['wrist']['vertical_deg'] = float('${W_VERTICAL:-90}')
d['wrist']['min_deg'] = float('${W_MIN_DEG:-5}')
d['wrist']['max_deg'] = float('${W_MAX_DEG:-300}')
d['wrist']['acceleration_deg_s2'] = float('${W_ACCEL:-180}')
d['wrist']['control_velocity_deg_s'] = float('${W_CTRL_VEL:-45}')
d['wrist']['position_tolerance_deg'] = float('${W_TOLERANCE:-4}')
d['wrist']['move_timeout_s'] = float('${W_TIMEOUT:-8}')
d['wrist']['return_after_open_delay_s'] = float('${W_RETURN_DELAY:-3}')
d['wrist']['vertical_delay_s'] = float('${W_VERT_DELAY:-3}')
with open('$tmp','w') as f:
    yaml.dump(d, f, default_flow_style=False, sort_keys=False)
" 2>/dev/null; then
        mv -f "$tmp" "$yaml"
        write_env_file
        echo "Saved to $yaml and /tmp/prosthesis_test_env.sh"
    else
        rm -f "$tmp"
        echo "Failed to save $yaml" >&2
        return 1
    fi
}

# ── Simple inline editor ─────────────────────────────────────────────
edit_var() {
    local label="$1" varname="$2"
    printf '\e[J'
    printf "\n  ${BOLD}%s${RESET}\n" "$label"
    printf "  Current: ${CYAN}%s${RESET}\n" "${!varname}"
    printf "  New: "
    show_cursor; read -r new_val; hide_cursor
    [ -n "$new_val" ] && printf -v "$varname" '%s' "$new_val"
}

# ── Parameter editor — test-grasp ────────────────────────────────────
param_editor_grasp() {
    _load_yaml_defaults
    local PARAM_LABELS=(
        "── Runtime ──"              ""
        "Main loop rate (Hz)"        CONTROL_RATE_HZ
        "CSV sample rate (Hz)"       CSV_RATE_HZ
        "Haptic update rate (Hz)"    HAPTICS_RATE_HZ
        "Terminal refresh (Hz)"      TERMINAL_RATE_HZ
        "Startup timeout (s)"        STARTUP_TIMEOUT_S
        "Auto-kill (s)"              AUTO_KILL_S
        "Mock hardware"              MOCK_HARDWARE
        "Use multi-node stack"       USE_MULTI_NODE
        "── Hand: Open ──"           ""
        "Open thumb (rad)"           OPEN_THUMB
        "Open index (rad)"           OPEN_INDEX
        "Open MRL (rad)"             OPEN_MRL
        "Open tolerance (rad)"       OPEN_TOLERANCE
        "Open min time (s)"          OPEN_MIN_S
        "Open timeout (s)"           OPEN_TIMEOUT_S
        "── Hand: Closure ──"        ""
        "Close vel start (rad/s)"    CLOSE_VEL_START
        "Close vel end (rad/s)"      CLOSE_VEL_END
        "Decay steps"                CLOSE_DECAY
        "Step interval (s)"          CLOSE_INTERVAL
        "Max close thumb (rad)"      MAXCLOSE_THUMB
        "Max close index (rad)"      MAXCLOSE_INDEX
        "Max close MRL (rad)"        MAXCLOSE_MRL
        "── Hand: Hold ──"           ""
        "Hold deadzone"              HOLD_DEADZONE
        "Hold min overshoot"         HOLD_MIN_OVER
        "Hold max overshoot"         HOLD_MAX_OVER
        "Hold max velocity (rad/s)"  HOLD_MAX_VEL
        "── Force ──"                ""
        "Contact thumb (raw)"        F_THUMB
        "Contact index (raw)"        F_INDEX
        "Contact MRL (raw)"          F_MRL
        "Init hold thumb (raw)"      F_INIT_THUMB
        "Init hold index (raw)"      F_INIT_INDEX
        "Init hold MRL (raw)"        F_INIT_MRL
        "Target min (raw)"           F_TARGET_MIN
        "Target max (raw)"           F_TARGET_MAX
        "Adj rate up (raw/s)"        F_ADJ_UP
        "Adj rate down (raw/s)"      F_ADJ_DOWN
        "Emergency threshold"        F_EMERGENCY
        "Backoff vel (rad/s)"        F_BACKOFF
        "── EMG ──"                  ""
        "Board IP"                   EMG_BOARD_IP
        "Confidence threshold"       EMG_CONF
        "Activation hold (s)"        EMG_ACTIVATION_HOLD
        "Open hold (s)"              EMG_OPEN_HOLD
        "Toggle hold (s)"            EMG_TOGGLE_HOLD
        "Stale timeout (s)"          EMG_STALE_TIMEOUT
        "Horizontal angle (deg)"     W_HORIZONTAL
        "Vertical angle (deg)"       W_VERTICAL
        "Min angle (deg)"            W_MIN_DEG
        "Max angle (deg)"            W_MAX_DEG
        "Acceleration (deg/s²)"      W_ACCEL
        "Control velocity (deg/s)"   W_CTRL_VEL
        "Tolerance (deg)"            W_TOLERANCE
        "Move timeout (s)"           W_TIMEOUT
        "Return delay (s)"           W_RETURN_DELAY
        "Vertical delay (s)"         W_VERT_DELAY
    )
    _param_editor "Haptic Force Test" PARAM_LABELS "save-on-start"
}

param_editor_emg_grasp() {
    local PARAM_LABELS=(
        "── Runtime ──"              ""
        "Auto-kill (s)"              AUTO_KILL_S
        "Mock hardware"              MOCK_HARDWARE
        "── EMG ──"                  ""
        "Board IP"                   EMG_BOARD_IP
        "Confidence threshold"       EMG_CONF
        "── Wrist ──"                ""
        "Min angle (deg)"            W_MIN_DEG
        "Max angle (deg)"            W_MAX_DEG
        "Acceleration (deg/s²)"      W_ACCEL
        "Control velocity (deg/s)"   W_CTRL_VEL
    )
    _param_editor "EMG Force Grasp" PARAM_LABELS
}

param_editor_emg_latency() {
    local PARAM_LABELS=(
        "── Runtime ──"              ""
        "Mia port"                   MIA_PORT
        "── EMG ──"                  ""
        "Board IP"                   EMG_BOARD_IP
        "Data dir"                   EMG_DATA_DIR
        "Model dir"                  EMG_MODEL_DIR
    )
    _param_editor "EMG Latency Benchmark" PARAM_LABELS
}

# ── Generic parameter editor ─────────────────────────────────────────
_param_editor() {
    local title="$1"; shift
    local -n labels="$1"; shift
    local save_on_start="${1:-}"
    local num_p=$(( ${#labels[@]} / 2 ))
    local sel=0 SAVED=0

    while true; do
        # Render
        local visible_start=0
        local term_h=$(tput lines 2>/dev/null || echo 40)
        local menu_h=$((term_h - 8))
        [ "$menu_h" -lt 10 ] && menu_h=30
        # Scroll to keep selection visible
        if [ $sel -ge $((visible_start + menu_h - 2)) ]; then
            visible_start=$((sel - menu_h + 3))
        elif [ $sel -lt $visible_start ]; then
            visible_start=$sel
        fi
        [ $visible_start -lt 0 ] && visible_start=0

        clear_screen
        echo "╭──────────────────────────────────────────────────────────╮"
        printf "│  ${BOLD}%s — Parameters${RESET}\n" "$title"
        echo "├──────────────────────────────────────────────────────────┤"
        printf "│  ${DIM}space${RESET}=start test  ${DIM}enter${RESET}=edit  ${DIM}s${RESET}=save  ${DIM}↑↓${RESET}=nav  ${DIM}q${RESET}=back      │\n"
        echo "╰──────────────────────────────────────────────────────────╯"
        echo ""
        local shown=0
        for ((i=0; i<num_p; i++)); do
            [ $i -lt $visible_start ] && continue
            [ $shown -ge $menu_h ] && { printf "  ${DIM}… %d more${RESET}\n" $((num_p - i)); break; }
            local label="${labels[$((i*2))]}"
            local vname="${labels[$((i*2+1))]}"
            if [ -z "$vname" ]; then
                # Section header
                printf "  ${BOLD}${YELLOW}%s${RESET}\n" "$label"
            elif [ "$i" -eq "$sel" ]; then
                printf "  ${REV} %-30s = ${CYAN}%-20s${RESET} ${REV} ${RESET}\n" "$label" "${!vname}"
            else
                printf "   %-30s = ${CYAN}%-20s${RESET}\n" "$label" "${!vname}"
            fi
            ((shown++))
        done
        echo ""
        if [ "$SAVED" = "1" ]; then
            printf "  ${GREEN}✓ Saved to config + env${RESET}\n"
        else
            printf "  ${DIM}Press ${RESET}${BOLD}s${RESET}${DIM} to save permanently. Changes apply on next test run.${RESET}\n"
        fi

        local key; key=$(read_key)
        case "$key" in
            UP)
                # Skip section headers
                sel=$((sel - 1))
                while [ $sel -ge 0 ] && [ -z "${labels[$((sel*2+1))]}" ]; do sel=$((sel - 1)); done
                [ $sel -lt 0 ] && sel=$((num_p - 1))
                while [ $sel -ge 0 ] && [ -z "${labels[$((sel*2+1))]}" ]; do sel=$((sel - 1)); done
                ;;
            DOWN)
                sel=$((sel + 1))
                while [ $sel -lt $num_p ] && [ -z "${labels[$((sel*2+1))]}" ]; do sel=$((sel + 1)); done
                [ $sel -ge $num_p ] && sel=0
                while [ $sel -lt $num_p ] && [ -z "${labels[$((sel*2+1))]}" ]; do sel=$((sel + 1)); done
                ;;
            ENTER)
                local vname="${labels[$((sel*2+1))]}"
                [ -n "$vname" ] && edit_var "${labels[$((sel*2))]}" "$vname"
                ;;
            s|S)
                if save_config; then
                    SAVED=1
                else
                    SAVED=0
                fi
                ;;
            ' ')
                if [ "$save_on_start" = "save-on-start" ]; then
                    save_config || { SAVED=0; continue; }
                    SAVED=1
                else
                    write_env_file
                fi
                clear_screen; show_cursor
                echo "test-grasp" > /tmp/prosthesis_test_choice
                return 0
                ;;
            q|Q) return 1 ;;
        esac
    done
}

# ── Main menu ────────────────────────────────────────────────────────
draw_main() {
    local sel=$1
    clear_screen
    echo "╭──────────────────────────────────────────────────────────╮"
    printf "│          ${BOLD}Prosthesis Test Selector${RESET}                              │\n"
    echo "├──────────────────────────────────────────────────────────┤"
    printf "│  ${DIM}space${RESET}=start  ${DIM}e${RESET}=params  ${DIM}↑↓${RESET}=navigate  ${DIM}q${RESET}=quit                 │\n"
    echo "╰──────────────────────────────────────────────────────────╯"
    echo ""
    for ((i=0; i<NUM_TESTS; i++)); do
        local label="${TESTS_LABEL[$i]}"
        local target="${TESTS_TARGET[$i]}"
        local has_p="${TESTS_PARAMS[$i]}"
        local tag=""
        [ "$has_p" = "yes" ] && tag=" ${DIM}[e=edit params]${RESET}"
        if [ "$i" -eq "$sel" ]; then
            printf "  ${REV} %-45s ${RESET}${DIM}→ make %s${RESET}%s\n" "$label" "$target" "$tag"
        else
            printf "   %-45s  ${DIM}→ make %s${RESET}%s\n" "$label" "$target" "$tag"
        fi
    done
    echo ""
    printf "  ${DIM}Press q to quit, space/enter to start the highlighted test.${RESET}\n"
}

run_menu() {
    _load_yaml_defaults
    local sel=0
    hide_cursor
    trap 'show_cursor; printf "\n"; exit 0' INT TERM
    while true; do
        draw_main "$sel"
        local key; key=$(read_key)
        case "$key" in
            UP)    sel=$(( (sel - 1 + NUM_TESTS) % NUM_TESTS )) ;;
            DOWN)  sel=$(( (sel + 1) % NUM_TESTS )) ;;
            e|E)
                local target="${TESTS_TARGET[$sel]}"
                case "$target" in
                    test-grasp)       param_editor_grasp && { write_env_file; clear_screen; show_cursor; echo "$target" > /tmp/prosthesis_test_choice; return 0; } || true ;;
                    emg-force-grasp)  param_editor_emg_grasp && { write_env_file; clear_screen; show_cursor; echo "$target" > /tmp/prosthesis_test_choice; return 0; } || true ;;
                    test-emg-latency) param_editor_emg_latency && { write_env_file; clear_screen; show_cursor; echo "$target" > /tmp/prosthesis_test_choice; return 0; } || true ;;
                esac
                ;;
            ' '|ENTER)
                write_env_file
                clear_screen; show_cursor
                echo "${TESTS_TARGET[$sel]}" > /tmp/prosthesis_test_choice
                return 0
                ;;
            q|Q) clear_screen; show_cursor; echo "Bye."; return 1 ;;
        esac
    done
}

if [ "${MENU_TEST_NO_RUN:-false}" != "true" ]; then
    run_menu
fi
