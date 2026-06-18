"""Shared constants for mia_haptic_force_test nodes.

All values mirror the defaults from the legacy monolithic script
(``scripts/mia_haptic_force_test.py``) and its config
(``config/mia_haptic_force_test.yaml``).  Topic names follow the new
multi-node contract defined by bead *mvp-7jf*; where they differ from the
legacy names the brief takes precedence.
"""

from __future__ import annotations

from enum import Enum
from typing import Final


# ── Hand geometry ───────────────────────────────────────────────────────────

FINGER_JOINTS: Final[list[str]] = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
"""Joint names for the three flexion joints (thumb, index, mrl)."""

FINGER_LABELS: Final[list[str]] = ["thumb", "index", "mrl"]
"""Canonical finger labels matching joint-index order."""

FINGER_COUNT: Final[int] = 3
"""Number of force-sensing fingers."""

MOTOR_COUNT: Final[int] = 8
"""Number of haptic band motors."""


# ── Controller names ────────────────────────────────────────────────────────

POSITION_CONTROLLERS: Final[list[str]] = [
    "group_pos_ff_controller",
    "thumb_pos_ff_controller",
    "index_pos_ff_controller",
    "mrl_pos_ff_controller",
]
"""Controllers that accept position commands (feed-forward)."""

VELOCITY_CONTROLLERS: Final[list[str]] = [
    "group_vel_ff_controller",
    "thumb_vel_ff_controller",
    "index_vel_ff_controller",
    "mrl_vel_ff_controller",
]
"""Controllers that accept velocity commands (feed-forward)."""


# ── Enums ───────────────────────────────────────────────────────────────────

class Stage(str, Enum):
    """Test lifecycle stages mirroring the legacy monolithic script.

    Values are **not** prefixed so they can be used directly as the payload
    for ``/test/stage``.
    """

    INITIALISING = "initialising"
    WAITING_FOR_ACTIVATION = "waiting_for_activation"
    ROTATING_TO_VERTICAL = "rotating_to_vertical"
    VERTICAL_DELAY = "vertical_delay"
    FORCE_CLOSING = "force_closing"
    FORCE_HOLD = "force_hold"
    OPENING_HAND = "opening_hand"
    RETURN_DELAY = "return_delay"
    RETURN_WRIST = "return_wrist"
    COMPLETE = "complete"
    FAULT = "fault"


class HoldControl(str, Enum):
    """Which hold modality the controller is applying."""

    FORCE = "force"
    WRIST = "wrist"


# ── EMG label constants ─────────────────────────────────────────────────────
# These are the default hardware label values from the legacy config's
# ``emg:`` section.  They may be overridden at runtime.

EMG_REST_LABEL: Final[int] = 0
EMG_ACTIVATION_LABEL: Final[int] = 1
EMG_WRIST_TOGGLE_LABEL: Final[int] = 1
EMG_OPEN_LABEL: Final[int] = 2
EMG_INCREASE_FORCE_LABEL: Final[int] = 3
EMG_DECREASE_FORCE_LABEL: Final[int] = 4
EMG_WRIST_POSITIVE_LABEL: Final[int] = 3
EMG_WRIST_NEGATIVE_LABEL: Final[int] = 4

GESTURES: Final[dict[str, int]] = {
    "REST": EMG_REST_LABEL,
    "POWER": EMG_ACTIVATION_LABEL,
    "OPEN": EMG_OPEN_LABEL,
    "FLEXION": EMG_INCREASE_FORCE_LABEL,
    "EXTENSION": EMG_DECREASE_FORCE_LABEL,
}
"""Default mapping from gesture name to numeric label."""

REST_GESTURE_LABEL: Final[int] = EMG_REST_LABEL


# ── Default topic names ─────────────────────────────────────────────────────
# Multi-node topic contract.  These are the canonical inter-node topics;
# nodes should treat them as defaults and allow config-file overrides.

# Hand state
TOPIC_HAND_JOINT_STATES: Final[str] = "/hand/joint_states"
TOPIC_HAND_FORCES: Final[str] = "/hand/forces"
TOPIC_HAND_FORCE_SOURCE: Final[str] = "/hand/force_source"

# EMG
TOPIC_EMG_GESTURE: Final[str] = "/emg/gesture"
TOPIC_EMG_GESTURE_LABEL: Final[str] = "/emg/gesture_label"
TOPIC_EMG_CONFIDENCE: Final[str] = "/emg/confidence"
TOPIC_EMG_PROPORTIONAL: Final[str] = "/emg/proportional"

# Control
TOPIC_CONTROL_TARGET_FORCE: Final[str] = "/control/target_force"
TOPIC_CONTROL_TARGET_WRIST: Final[str] = "/control/target_wrist"
TOPIC_CONTROL_MODE: Final[str] = "/control/mode"
TOPIC_CONTROL_ENABLE: Final[str] = "/control/enable"
TOPIC_CONTROL_HOLD_MODE: Final[str] = "/control/hold_mode"

# Test orchestration
TOPIC_TEST_STAGE: Final[str] = "/test/stage"
TOPIC_TEST_EVENT: Final[str] = "/test/event"
TOPIC_TEST_STATUS: Final[str] = "/test/status"

# Controller feedback
TOPIC_CONTROLLER_FORCE_ERROR: Final[str] = "/controller/force_error"
TOPIC_CONTROLLER_ACTIVE: Final[str] = "/controller/active"
TOPIC_CONTROLLER_LOOP_TIMING: Final[str] = "/controller/loop_timing"

# Haptic band
TOPIC_HAPTIC_BAND_MOTORS: Final[str] = "/haptic_band/motors"

# Hardware command topics (the hand-driver controllers)
TOPIC_GROUP_VEL_FF_COMMANDS: Final[str] = "/group_vel_ff_controller/commands"
TOPIC_GROUP_POS_FF_COMMANDS: Final[str] = "/group_pos_ff_controller/commands"

# Wrist
TOPIC_WRIST_SET_POSITION: Final[str] = "/wrist/set_position"
TOPIC_WRIST_STATE: Final[str] = "/wrist/state"

# Hardware input streams (raw Mia Hand topics consumed by input-processing
# nodes; the controller itself subscribes to the processed /hand/* topics).
TOPIC_HW_FINGER_FORCES: Final[str] = "data_streams/fingers/forces/data"
TOPIC_HW_MOTOR_POSITIONS: Final[str] = "data_streams/motors/positions/data"
TOPIC_HW_MOTOR_SPEEDS: Final[str] = "data_streams/motors/speeds/data"
TOPIC_HW_MOTOR_CURRENTS: Final[str] = "data_streams/motors/currents/data"
TOPIC_HW_JOINT_POSITIONS: Final[str] = "data_streams/joints/positions/data"
TOPIC_HW_JOINT_SPEEDS: Final[str] = "data_streams/joints/speeds/data"
TOPIC_HW_JOINT_STATES: Final[str] = "/joint_states"


# ── Message-type string constants (for doc / introspection) ─────────────────

MSG_JOINT_STATE: Final[str] = "sensor_msgs/JointState"
MSG_FLOAT32_MULTI: Final[str] = "std_msgs/Float32MultiArray"
MSG_FLOAT64_MULTI: Final[str] = "std_msgs/Float64MultiArray"
MSG_STRING: Final[str] = "std_msgs/String"
MSG_INT32: Final[str] = "std_msgs/Int32"
MSG_FLOAT32: Final[str] = "std_msgs/Float32"
MSG_FLOAT64: Final[str] = "std_msgs/Float64"
MSG_BOOL: Final[str] = "std_msgs/Bool"
MSG_FORCE_DATA: Final[str] = "mia_hand_msgs/ForceData"
MSG_MOTOR_DATA: Final[str] = "mia_hand_msgs/MotorData"
MSG_JOINT_DATA: Final[str] = "mia_hand_msgs/JointData"
