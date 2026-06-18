"""Deterministic hand/wrist simulator for offline testing.

The legacy ``scripts/mia_haptic_force_test.py`` script and the split-node
stack are both tested against real hardware.  The ``HandSimulator`` here
mirrors the relevant part of the hand's behaviour well enough to drive the
supervisor/controller/haptic/log stack without any physical device:

* finger positions move with the commanded velocity/position until they
  cross a per-finger contact point, then normal force rises linearly with
  penetration up to a configurable maximum;
* wrist position slews toward the commanded target with a configurable
  acceleration and maximum velocity;
* effort values mirror the synthetic normal force so the controller
  logging fallback path has real data;
* the simulator publishes joint states on a topic that the rest of the
  stack can be pointed at (instead of the real ``/joint_states``).

The simulator is intentionally **deterministic**: no noise, no random
phases, no clock drift.  That makes the offline test suite reproducible
while still triggering the same hold/fault branches as the real hand.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Sequence

from .conversions import clamp
from .constants import FINGER_COUNT, FINGER_JOINTS


# ── Default simulation parameters ───────────────────────────────────────────
# These mirror the legacy config values in
# ``config/mia_haptic_force_test.yaml`` and are used when the helper is
# invoked without an explicit cfg override.

DEFAULT_FINGER_OPEN_RAD: tuple[float, ...] = (0.0, 0.0, 0.0)
DEFAULT_FINGER_MAX_CLOSURE_RAD: tuple[float, ...] = (1.5, 1.5, 1.5)
DEFAULT_FINGER_CONTACT_RAD: tuple[float, ...] = (0.4, 0.4, 0.4)
DEFAULT_FINGER_STIFFNESS_N_PER_RAD: tuple[float, ...] = (1500.0, 1500.0, 1500.0)
DEFAULT_FINGER_MAX_FORCE_N: tuple[float, ...] = (500.0, 500.0, 500.0)
DEFAULT_FINGER_IDLE_FORCE_N: float = 0.0

DEFAULT_WRIST_OPEN_DEG: float = 180.0
DEFAULT_WRIST_MAX_VELOCITY_DEG_S: float = 60.0
DEFAULT_WRIST_ACCELERATION_DEG_S2: float = 45.0
DEFAULT_WRIST_MIN_DEG: float = 0.0
DEFAULT_WRIST_MAX_DEG: float = 360.0


# ── State dataclass ─────────────────────────────────────────────────────────


@dataclass(frozen=True)
class HandSimState:
    """Pure value type representing one step of the simulated hand.

    All per-finger data is stored in canonical order (thumb, index, mrl) and
    the wrist state lives alongside it.  Frozen so test code can safely
    compare snapshots without copying.
    """

    finger_positions_rad: tuple[float, ...] = DEFAULT_FINGER_OPEN_RAD
    finger_velocities_rad_s: tuple[float, ...] = (0.0, 0.0, 0.0)
    finger_forces_n: tuple[float, ...] = (0.0, 0.0, 0.0)
    finger_contacted: tuple[bool, ...] = (False, False, False)
    wrist_position_deg: float = DEFAULT_WRIST_OPEN_DEG
    wrist_velocity_deg_s: float = 0.0
    step_count: int = 0

    def finger_position(self, index: int) -> float:
        return self.finger_positions_rad[index]

    def finger_velocity(self, index: int) -> float:
        return self.finger_velocities_rad_s[index]

    def finger_force(self, index: int) -> float:
        return self.finger_forces_n[index]


# ── Configuration helpers ──────────────────────────────────────────────────


def _finger_cfg(section: Any, default: float) -> tuple[float, ...]:
    """Coerce a per-finger config section into a ``(thumb, index, mrl)`` tuple.

    Accepts a dict keyed by joint name (legacy format) or a list/tuple of
    length ``FINGER_COUNT``.  Anything else falls back to ``default``.
    """
    if isinstance(section, dict):
        return tuple(float(section.get(name, default)) for name in FINGER_JOINTS)
    if isinstance(section, (list, tuple)):
        values = [float(v) for v in section]
        values += [default] * (FINGER_COUNT - len(values))
        return tuple(values[:FINGER_COUNT])
    return tuple(float(default) for _ in range(FINGER_COUNT))


def sim_config_from(raw_cfg: dict[str, Any]) -> dict[str, Any]:
    hand = raw_cfg.get("hand", {}) if isinstance(raw_cfg, dict) else {}
    wrist = raw_cfg.get("wrist", {}) if isinstance(raw_cfg, dict) else {}
    if not isinstance(hand, dict):
        hand = {}
    if not isinstance(wrist, dict):
        wrist = {}
    return {
        "open_rad": _finger_cfg(hand.get("open_positions", {}), 0.0),
        "max_closure_rad": _finger_cfg(hand.get("max_closure_positions", {}), 1.5),
        "contact_rad": _finger_cfg(hand.get("contact_positions_rad", {}), 0.4),
        "stiffness_n_per_rad": _finger_cfg(hand.get("stiffness_n_per_rad", {}), 1500.0),
        "max_force_n": _finger_cfg(hand.get("max_force_n", {}), 500.0),
        "idle_force_n": float(hand.get("idle_force_n", DEFAULT_FINGER_IDLE_FORCE_N)),
        "wrist_open_deg": float(wrist.get("horizontal_deg", DEFAULT_WRIST_OPEN_DEG)),
        "wrist_max_velocity_deg_s": float(
            wrist.get("control_velocity_deg_s", DEFAULT_WRIST_MAX_VELOCITY_DEG_S)
        ),
        "wrist_acceleration_deg_s2": float(
            wrist.get("acceleration_deg_s2", DEFAULT_WRIST_ACCELERATION_DEG_S2)
        ),
        "wrist_min_deg": float(wrist.get("min_deg", DEFAULT_WRIST_MIN_DEG)),
        "wrist_max_deg": float(wrist.get("max_deg", DEFAULT_WRIST_MAX_DEG)),
    }

def default_sim_config() -> dict[str, Any]:
    """Return a simulator config populated with the module defaults."""
    return {
        "open_rad": DEFAULT_FINGER_OPEN_RAD,
        "max_closure_rad": DEFAULT_FINGER_MAX_CLOSURE_RAD,
        "contact_rad": DEFAULT_FINGER_CONTACT_RAD,
        "stiffness_n_per_rad": DEFAULT_FINGER_STIFFNESS_N_PER_RAD,
        "max_force_n": DEFAULT_FINGER_MAX_FORCE_N,
        "idle_force_n": DEFAULT_FINGER_IDLE_FORCE_N,
        "wrist_open_deg": DEFAULT_WRIST_OPEN_DEG,
        "wrist_max_velocity_deg_s": DEFAULT_WRIST_MAX_VELOCITY_DEG_S,
        "wrist_acceleration_deg_s2": DEFAULT_WRIST_ACCELERATION_DEG_S2,
        "wrist_min_deg": DEFAULT_WRIST_MIN_DEG,
        "wrist_max_deg": DEFAULT_WRIST_MAX_DEG,
    }


# ── Core step function ─────────────────────────────────────────────────────


def _resolve_finger_command(
    target: Sequence[float] | None,
    fallback_values: tuple[float, ...],
) -> tuple[float, ...]:
    """Coerce *target* to per-finger floats, or return *fallback_values*.

    * ``None`` → ``fallback_values`` (no command, keep previous state).
    * Empty sequence → all zeros.
    * Sequence shorter than ``FINGER_COUNT`` → missing entries default to 0.0.
    * Non-numeric entries default to 0.0.
    """
    if target is None:
        return fallback_values
    out: list[float] = []
    for i in range(FINGER_COUNT):
        if i < len(target):
            try:
                out.append(float(target[i]))
            except (TypeError, ValueError):
                out.append(0.0)
        else:
            out.append(0.0)
    return tuple(out)


def _step_finger(
    *,
    position: float,
    velocity: float,
    force: float,
    contacted: bool,
    pos_cmd: float,
    vel_cmd: float,
    dt_s: float,
    contact_rad: float,
    stiffness_n_per_rad: float,
    max_force_n: float,
    max_closure_rad: float,
    idle_force_n: float,
) -> tuple[float, float, float, bool]:
    """Advance one finger one step.

    *vel_cmd* is the commanded velocity in rad/s and *pos_cmd* is the
    commanded position in rad.  Exactly one of them is expected to be
    meaningful per call (the controller switches modes), but both are
    applied when given: the position command is treated as a soft target
    and the velocity command drives the integration.

    The returned tuple is ``(new_position, new_velocity, new_force,
    new_contacted)``.
    """
    # Integrate position.  The velocity command is the primary driver; the
    # position command is honoured when the velocity is zero.
    if abs(vel_cmd) > 1e-6:
        new_position = clamp(position + vel_cmd * dt_s, 0.0, max_closure_rad)
        new_velocity = vel_cmd
    elif abs(pos_cmd - position) > 1e-6:
        new_position = clamp(pos_cmd, 0.0, max_closure_rad)
        new_velocity = 0.0
    else:
        new_position = clamp(position, 0.0, max_closure_rad)
        new_velocity = 0.0

    # Spring/contact model.
    penetration = new_position - contact_rad
    if penetration > 0.0:
        new_force = clamp(stiffness_n_per_rad * penetration, 0.0, max_force_n)
        new_contacted = True
    else:
        new_force = float(idle_force_n)
        new_contacted = False

    return new_position, new_velocity, new_force, new_contacted


def step_hand_simulation(
    state: HandSimState,
    *,
    finger_vel_cmd: Sequence[float] | None = None,
    finger_pos_cmd: Sequence[float] | None = None,
    wrist_target_deg: float | None = None,
    wrist_max_velocity_deg_s: float | None = None,
    wrist_acceleration_deg_s2: float | None = None,
    dt_s: float = 0.01,
    cfg: dict[str, Any] | None = None,
) -> HandSimState:
    """Advance the simulated hand by ``dt_s`` and return the new state.

    The helper is pure: the input state is never mutated.  Caller
    supplies the per-finger velocity/position command (typically forwarded
    from the controller) and an optional wrist target; ``None`` leaves the
    corresponding subsystem alone.
    """
    sim_cfg: dict[str, Any] = {**default_sim_config(), **(cfg or {})}

    vel_cmds = _resolve_finger_command(
        finger_vel_cmd,
        state.finger_velocities_rad_s,
    )
    pos_cmds = _resolve_finger_command(
        finger_pos_cmd,
        state.finger_positions_rad,
    )

    new_positions: list[float] = []
    new_velocities: list[float] = []
    new_forces: list[float] = []
    new_contacted: list[bool] = []
    for i in range(FINGER_COUNT):
        pos, vel, force, contacted = _step_finger(
            position=state.finger_positions_rad[i],
            velocity=state.finger_velocities_rad_s[i],
            force=state.finger_forces_n[i],
            contacted=state.finger_contacted[i],
            pos_cmd=pos_cmds[i],
            vel_cmd=vel_cmds[i],
            dt_s=dt_s,
            contact_rad=sim_cfg["contact_rad"][i],
            stiffness_n_per_rad=sim_cfg["stiffness_n_per_rad"][i],
            max_force_n=sim_cfg["max_force_n"][i],
            max_closure_rad=sim_cfg["max_closure_rad"][i],
            idle_force_n=sim_cfg["idle_force_n"],
        )
        new_positions.append(pos)
        new_velocities.append(vel)
        new_forces.append(force)
        new_contacted.append(contacted)

    # ── Wrist integration (trapezoidal velocity profile) ──────────────────
    if wrist_target_deg is not None:
        max_vel = float(
            wrist_max_velocity_deg_s
            if wrist_max_velocity_deg_s is not None
            else sim_cfg["wrist_max_velocity_deg_s"]
        )
        accel = float(
            wrist_acceleration_deg_s2
            if wrist_acceleration_deg_s2 is not None
            else sim_cfg["wrist_acceleration_deg_s2"]
        )
        min_deg = float(sim_cfg["wrist_min_deg"])
        max_deg = float(sim_cfg["wrist_max_deg"])
        target = clamp(wrist_target_deg, min_deg, max_deg)
        err = target - state.wrist_position_deg
        max_step = max_vel * dt_s
        max_accel_step = accel * dt_s * dt_s
        step = clamp(err, -max_step - max_accel_step, max_step + max_accel_step)
        new_wrist_position = clamp(
            state.wrist_position_deg + step, min_deg, max_deg
        )
        new_wrist_velocity = (new_wrist_position - state.wrist_position_deg) / dt_s
    else:
        new_wrist_position = state.wrist_position_deg
        new_wrist_velocity = state.wrist_velocity_deg_s

    return replace(
        state,
        finger_positions_rad=tuple(new_positions),
        finger_velocities_rad_s=tuple(new_velocities),
        finger_forces_n=tuple(new_forces),
        finger_contacted=tuple(new_contacted),
        wrist_position_deg=new_wrist_position,
        wrist_velocity_deg_s=new_wrist_velocity,
        step_count=state.step_count + 1,
    )


# ── Convenience: open the hand back to rest ────────────────────────────────


def open_hand(
    state: HandSimState, cfg: dict[str, Any] | None = None
) -> HandSimState:
    """Return a state with the fingers at their open position and zero velocity.

    Useful for the supervisor's ``opening_hand`` stage and for resetting
    the simulator between test scenarios.
    """
    sim_cfg: dict[str, Any] = {**default_sim_config(), **(cfg or {})}
    open_positions = sim_cfg["open_rad"]
    return replace(
        state,
        finger_positions_rad=tuple(open_positions),
        finger_velocities_rad_s=(0.0, 0.0, 0.0),
        finger_forces_n=(float(sim_cfg["idle_force_n"]),) * FINGER_COUNT,
        finger_contacted=(False, False, False),
    )


__all__ = [
    "DEFAULT_FINGER_OPEN_RAD",
    "DEFAULT_FINGER_MAX_CLOSURE_RAD",
    "DEFAULT_FINGER_CONTACT_RAD",
    "DEFAULT_FINGER_STIFFNESS_N_PER_RAD",
    "DEFAULT_FINGER_MAX_FORCE_N",
    "DEFAULT_FINGER_IDLE_FORCE_N",
    "DEFAULT_WRIST_OPEN_DEG",
    "DEFAULT_WRIST_MAX_VELOCITY_DEG_S",
    "DEFAULT_WRIST_ACCELERATION_DEG_S2",
    "DEFAULT_WRIST_MIN_DEG",
    "DEFAULT_WRIST_MAX_DEG",
    "FINGER_COUNT",
    "FINGER_JOINTS",
    "HandSimState",
    "default_sim_config",
    "open_hand",
    "sim_config_from",
    "step_hand_simulation",
]
