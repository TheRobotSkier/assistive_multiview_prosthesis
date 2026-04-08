import os
import numpy as np
import pinocchio as pin

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "urdf", "mia_hand_flat.urdf"))
LUT_PATH = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "data", "finger_contact_lut.npz")
)

# Active motor order: [Thumb_Flex, TISIT_Motor, MRL_Flex]
low = np.array([0.0, -1.0, 0.0], dtype=float)
high = np.array([1.0, 1.0, 1.0], dtype=float)

JOINT_ORDER = ["mia_j_index_fle", "mia_j_mrl_fle", "mia_j_thumb_fle", "mia_j_thumb_opp"]

# Keep index/mrl/thumb flex in [0, 1] for this LUT stage, opposition in [0, 1] mapped to URDF bounds.
CONTROL_LOWER = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
CONTROL_UPPER = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)

# The thumb opposition axis is requested as two discrete modes for LUT.
THUMB_OPPOSITION_STATES = (0.0, 1.0)

CONTACT_GROUPS = {
    "index": [
        "mia_index_fle_0",
        "mia_index_sensor_0",
        "mia_index_sensor_1",
        "mia_index_sensor_2",
    ],
    "middle": [
        "mia_middle_fle_0",
        "mia_middle_sensor_0",
        "mia_middle_sensor_1",
        "mia_middle_sensor_2",
    ],
    "ring": [
        "mia_ring_fle_0",
        "mia_ring_fle_1",
        "mia_ring_fle_2",
    ],
    "little": [
        "mia_little_fle_0",
        "mia_little_fle_1",
        "mia_little_fle_2",
    ],
    "thumb": [
        "mia_thumb_fle_0",
        "mia_thumb_fle_1",
        "mia_thumb_fle_2",
    ],
    "palm": ["mia_palm_0", "mia_palm_1"],
}

CONTACT_DEFINITIONS = [
    {"name": "index_proximal_pad", "group": "index", "geom": "mia_index_fle_0", "offset": "cyl_mid_palmar"},
    {"name": "index_mid_pad", "group": "index", "geom": "mia_index_sensor_0", "offset": "cyl_mid_palmar"},
    {"name": "index_distal_pad", "group": "index", "geom": "mia_index_sensor_1", "offset": "cyl_distal_palmar"},
    {"name": "index_tip_pad", "group": "index", "geom": "mia_index_sensor_2", "offset": "sphere_pad"},
    {"name": "middle_proximal_pad", "group": "middle", "geom": "mia_middle_fle_0", "offset": "cyl_mid_palmar"},
    {"name": "middle_mid_pad", "group": "middle", "geom": "mia_middle_sensor_0", "offset": "cyl_mid_palmar"},
    {"name": "middle_distal_pad", "group": "middle", "geom": "mia_middle_sensor_1", "offset": "cyl_distal_palmar"},
    {"name": "middle_tip_pad", "group": "middle", "geom": "mia_middle_sensor_2", "offset": "sphere_pad"},
    {"name": "ring_proximal_pad", "group": "ring", "geom": "mia_ring_fle_1", "offset": "cyl_mid_palmar"},
    {"name": "ring_mid_pad", "group": "ring", "geom": "mia_ring_fle_0", "offset": "cyl_mid_palmar"},
    {"name": "ring_lateral_pad", "group": "ring", "geom": "mia_ring_fle_0", "offset": "cyl_side_palmar"},
    {"name": "ring_tip_pad", "group": "ring", "geom": "mia_ring_fle_2", "offset": "sphere_pad"},
    {"name": "little_proximal_pad", "group": "little", "geom": "mia_little_fle_1", "offset": "cyl_mid_palmar"},
    {"name": "little_mid_pad", "group": "little", "geom": "mia_little_fle_0", "offset": "cyl_mid_palmar"},
    {"name": "little_lateral_pad", "group": "little", "geom": "mia_little_fle_0", "offset": "cyl_side_palmar"},
    {"name": "little_tip_pad", "group": "little", "geom": "mia_little_fle_2", "offset": "sphere_pad"},
    {"name": "thumb_proximal_pad", "group": "thumb", "geom": "mia_thumb_fle_1", "offset": "cyl_mid_palmar"},
    {"name": "thumb_mid_pad", "group": "thumb", "geom": "mia_thumb_fle_0", "offset": "cyl_mid_palmar"},
    {"name": "thumb_precision_pad", "group": "thumb", "geom": "mia_thumb_fle_0", "offset": "cyl_distal_palmar"},
    {"name": "thumb_tip_pad", "group": "thumb", "geom": "mia_thumb_fle_2", "offset": "sphere_pad"},
    {"name": "palm_distal_center", "group": "palm", "geom": "mia_palm_0", "offset": "box_palmar_center"},
    {"name": "palm_distal_right", "group": "palm", "geom": "mia_palm_0", "offset": "box_palmar_right"},
    {"name": "palm_distal_left", "group": "palm", "geom": "mia_palm_0", "offset": "box_palmar_left"},
    {"name": "palm_distal_side", "group": "palm", "geom": "mia_palm_0", "offset": "box_side_support"},
    {"name": "palm_proximal_center", "group": "palm", "geom": "mia_palm_1", "offset": "box_palmar_center"},
    {"name": "palm_proximal_side", "group": "palm", "geom": "mia_palm_1", "offset": "box_side_support"},
]


def _build_model_bundle(urdf_path):
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"Could not find URDF at: {urdf_path}")

    m = pin.buildModelFromUrdf(urdf_path)
    d = m.createData()
    g = pin.buildGeomFromUrdf(m, urdf_path, pin.GeometryType.COLLISION)
    return m, d, g


model, data, geom_model = _build_model_bundle(URDF_PATH)

def _extract_runtime_joint_limits(m):
    limits = {}
    for j_name in JOINT_ORDER:
        j_id = m.getJointId(j_name)
        q_idx = j_id - 1
        limits[j_name] = (
            float(m.lowerPositionLimit[q_idx]),
            float(m.upperPositionLimit[q_idx]),
        )
    return limits


def _extract_collision_geometries(g_model, groups):
    geometries = {}
    for group_names in groups.values():
        for geom_name in group_names:
            if geom_name in geometries:
                continue
            if not g_model.existGeometryName(geom_name):
                raise RuntimeError(f"Missing collision geometry: {geom_name}")
            geom_id = g_model.getGeometryId(geom_name)
            geom_obj = g_model.geometryObjects[geom_id]
            geom = geom_obj.geometry

            if hasattr(geom, "radius") and hasattr(geom, "halfLength"):
                geom_type = "cylinder"
                params = {
                    "radius": float(geom.radius),
                    "length": float(2.0 * geom.halfLength),
                }
            elif hasattr(geom, "radius"):
                geom_type = "sphere"
                params = {"radius": float(geom.radius)}
            elif hasattr(geom, "halfSide"):
                geom_type = "box"
                params = {"half_extents": np.array(geom.halfSide, dtype=float)}
            else:
                raise RuntimeError(f"Unsupported collision primitive type for {geom_name}")

            geometries[geom_name] = {
                "joint_id": geom_obj.parentJoint,
                "placement": geom_obj.placement,
                "type": geom_type,
                "params": params,
            }
    return geometries


JOINT_LIMITS = _extract_runtime_joint_limits(model)

# Keep control mapping realistic while aligned to synergy controls.
JOINT_LIMITS_FOR_CONTROL = {
    "mia_j_index_fle": (0.0, JOINT_LIMITS["mia_j_index_fle"][1]),
    "mia_j_mrl_fle": (0.0, JOINT_LIMITS["mia_j_mrl_fle"][1]),
    "mia_j_thumb_fle": (0.0, JOINT_LIMITS["mia_j_thumb_fle"][1]),
    "mia_j_thumb_opp": JOINT_LIMITS["mia_j_thumb_opp"],
}

COLLISION_GEOMETRIES = _extract_collision_geometries(geom_model, CONTACT_GROUPS)


def _offset_for_strategy(geom_info, strategy_name):
    geom_type = geom_info["type"]
    params = geom_info["params"]

    if geom_type == "cylinder":
        r = params["radius"]
        l = params["length"]
        if strategy_name == "cyl_mid_palmar":
            return np.array([0.0, -1.05 * r, 0.0], dtype=float)
        if strategy_name == "cyl_distal_palmar":
            return np.array([0.0, -1.05 * r, 0.35 * l], dtype=float)
        if strategy_name == "cyl_side_palmar":
            return np.array([0.65 * r, -0.8 * r, 0.0], dtype=float)

    if geom_type == "sphere":
        r = params["radius"]
        if strategy_name == "sphere_pad":
            return np.array([0.0, -1.05 * r, 0.2 * r], dtype=float)

    if geom_type == "box":
        hx, hy, hz = params["half_extents"]
        if strategy_name == "box_palmar_center":
            return np.array([0.0, 0.0, -1.02 * hz], dtype=float)
        if strategy_name == "box_palmar_right":
            return np.array([0.35 * hx, 0.0, -1.02 * hz], dtype=float)
        if strategy_name == "box_palmar_left":
            return np.array([-0.35 * hx, 0.0, -1.02 * hz], dtype=float)
        if strategy_name == "box_side_support":
            return np.array([0.0, 1.02 * hy, -0.4 * hz], dtype=float)

    raise RuntimeError(
        f"Invalid sampling strategy '{strategy_name}' for geometry type '{geom_type}'"
    )


for contact in CONTACT_DEFINITIONS:
    geom_info = COLLISION_GEOMETRIES[contact["geom"]]
    contact["local_offset"] = _offset_for_strategy(geom_info, contact["offset"])


def _compute_palm_center_at_neutral():
    q_neutral = np.array([0.0, 0.0, 0.0], dtype=float)
    q_f = get_q_full(q_neutral)
    pin.forwardKinematics(model, data, q_f)

    palm_geoms = [c for c in CONTACT_DEFINITIONS if c["group"] == "palm"]
    points = []
    for c in palm_geoms:
        g = COLLISION_GEOMETRIES[c["geom"]]
        m_joint = data.oMi[g["joint_id"]]
        m_geom = m_joint * g["placement"]
        points.append(m_geom.translation + m_geom.rotation @ c["local_offset"])
    return np.mean(points, axis=0)


def _compute_neutral_reference_points():
    q_neutral = np.array([0.0, 0.0, 0.0], dtype=float)
    transforms = get_sampled_contact_transforms(q_neutral)
    palm_points = np.array(
        [transforms[c["name"]][:3, 3] for c in CONTACT_DEFINITIONS if c["group"] == "palm"],
        dtype=float,
    )
    return {
        "palm_center": np.mean(palm_points, axis=0),
        "index_tip": transforms["index_tip_pad"][:3, 3].copy(),
        "thumb_tip": transforms["thumb_tip_pad"][:3, 3].copy(),
    }


def _radial_candidates(radius, z_val=0.0, num=12):
    return [
        np.array([radius * np.cos(t), radius * np.sin(t), z_val], dtype=float)
        for t in np.linspace(0.0, 2.0 * np.pi, num=num, endpoint=False)
    ]


def _align_contact_offsets_to_targets():
    # Use neutral pose to pick radial directions based on the target interaction region.
    refs = _compute_neutral_reference_points()
    q_neutral = np.array([0.0, 0.0, 0.0], dtype=float)
    q_f = get_q_full(q_neutral)
    pin.forwardKinematics(model, data, q_f)

    for contact in CONTACT_DEFINITIONS:
        if contact["group"] == "palm":
            continue

        geom_info = COLLISION_GEOMETRIES[contact["geom"]]
        geom_type = geom_info["type"]
        params = geom_info["params"]

        m_joint = data.oMi[geom_info["joint_id"]]
        m_geom = m_joint * geom_info["placement"]
        if contact["group"] == "index":
            target_world = refs["thumb_tip"]
        elif contact["group"] == "thumb":
            target_world = refs["index_tip"]
        else:
            target_world = refs["palm_center"]

        v_to_target_world = target_world - m_geom.translation

        if np.linalg.norm(v_to_target_world) < 1e-10:
            continue

        if geom_type == "cylinder":
            r = params["radius"]
            z_val = float(contact["local_offset"][2])
            candidates_local = _radial_candidates(1.05 * r, z_val=z_val)
        elif geom_type == "sphere":
            r = params["radius"]
            # Keep a slight distal bias but select azimuth by palm-facing score.
            candidates_local = _radial_candidates(1.05 * r, z_val=0.2 * r)
        else:
            continue

        best = None
        best_score = -np.inf
        for cand_local in candidates_local:
            cand_world_vec = m_geom.rotation @ cand_local
            score = float(np.dot(cand_world_vec, v_to_target_world))
            if score > best_score:
                best_score = score
                best = cand_local

        if best is not None:
            contact["local_offset"] = best


CONTACT_NAMES_BY_GROUP = {
    group: [c["name"] for c in CONTACT_DEFINITIONS if c["group"] == group]
    for group in CONTACT_GROUPS.keys()
}

ALL_CONTACT_NAMES = [c["name"] for c in CONTACT_DEFINITIONS]


def _clip_unit_controls(ctrl):
    clipped = np.asarray(ctrl, dtype=float)
    return np.clip(clipped, CONTROL_LOWER, CONTROL_UPPER)


def _to_joint_value(normalized_value, joint_name):
    low_lim, high_lim = JOINT_LIMITS_FOR_CONTROL[joint_name]
    return low_lim + normalized_value * (high_lim - low_lim)


def _enforce_runtime_joint_limits(q_full):
    q_out = q_full.copy()
    for j_name in JOINT_ORDER:
        j_id = model.getJointId(j_name)
        q_idx = j_id - 1
        low_lim, high_lim = JOINT_LIMITS[j_name]
        q_out[q_idx] = np.clip(q_out[q_idx], low_lim, high_lim)
    return q_out


def _quat_mul_wxyz(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def _rotation_matrix_to_quat_wxyz(r):
    trace = float(np.trace(r))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s

    q = np.array([w, x, y, z], dtype=float)
    q /= np.linalg.norm(q)
    return q


def se3_matrix_to_dual_quaternion(transform):
    """Convert a 4x4 SE3 matrix to dual quaternion [qr_w..qd_z]."""
    rot = transform[:3, :3]
    trans = transform[:3, 3]

    q_r = _rotation_matrix_to_quat_wxyz(rot)
    q_t = np.array([0.0, trans[0], trans[1], trans[2]], dtype=float)
    q_d = 0.5 * _quat_mul_wxyz(q_t, q_r)
    return np.concatenate([q_r, q_d]).astype(np.float32)

def get_q_full(q_active):
    """Map active motors [Thumb_Flex, TISIT_Motor, MRL_Flex] into full joint vector."""
    q_active = np.asarray(q_active, dtype=float)
    q_full = np.zeros(model.nq)

    # TISIT coupling logic.
    m2_pos = q_active[1]
    if m2_pos <= 0.0:
        index_val = -m2_pos
        opp_val = 0.0
    else:
        index_val = m2_pos
        opp_val = m2_pos

    components = _clip_unit_controls([index_val, q_active[2], q_active[0], opp_val])

    q_joint_vals = np.array(
        [
            _to_joint_value(components[0], "mia_j_index_fle"),
            _to_joint_value(components[1], "mia_j_mrl_fle"),
            _to_joint_value(components[2], "mia_j_thumb_fle"),
            _to_joint_value(components[3], "mia_j_thumb_opp"),
        ],
        dtype=float,
    )

    q_full[model.getJointId("mia_j_thumb_fle") - 1] = q_joint_vals[2]

    q_full[model.getJointId("mia_j_index_fle") - 1] = q_joint_vals[0]
    q_full[model.getJointId("mia_j_thumb_opp") - 1] = q_joint_vals[3]

    q_full[model.getJointId("mia_j_mrl_fle") - 1] = q_joint_vals[1]
    q_full[model.getJointId("mia_j_ring_fle") - 1] = q_joint_vals[1]
    q_full[model.getJointId("mia_j_little_fle") - 1] = q_joint_vals[1]

    return _enforce_runtime_joint_limits(q_full)


def _q_full_with_thumb_mode(q_active, thumb_opp_mode):
    """Override thumb opposition with a two-state mode while keeping other controls unchanged."""
    q_active = np.asarray(q_active, dtype=float)

    m2_pos = q_active[1]
    if m2_pos <= 0.0:
        index_val = -m2_pos
    else:
        index_val = m2_pos

    q_full = np.zeros(model.nq)
    q_full[model.getJointId("mia_j_index_fle") - 1] = _to_joint_value(
        np.clip(index_val, 0.0, 1.0), "mia_j_index_fle"
    )
    q_full[model.getJointId("mia_j_mrl_fle") - 1] = _to_joint_value(
        np.clip(q_active[2], 0.0, 1.0), "mia_j_mrl_fle"
    )
    q_full[model.getJointId("mia_j_ring_fle") - 1] = q_full[model.getJointId("mia_j_mrl_fle") - 1]
    q_full[model.getJointId("mia_j_little_fle") - 1] = q_full[model.getJointId("mia_j_mrl_fle") - 1]
    q_full[model.getJointId("mia_j_thumb_fle") - 1] = _to_joint_value(
        np.clip(q_active[0], 0.0, 1.0), "mia_j_thumb_fle"
    )
    q_full[model.getJointId("mia_j_thumb_opp") - 1] = _to_joint_value(
        np.clip(float(thumb_opp_mode), 0.0, 1.0), "mia_j_thumb_opp"
    )
    return _enforce_runtime_joint_limits(q_full)


def get_all_finger_positions(q_active):
    """Return dict[finger] -> np.array([x, y, z]) in world coordinates."""
    transforms = get_sampled_contact_transforms(q_active)

    results = {}
    primary = {
        "index": "index_tip_pad",
        "middle": "middle_tip_pad",
        "ring": "ring_tip_pad",
        "little": "little_tip_pad",
        "thumb": "thumb_tip_pad",
    }
    for finger, contact_name in primary.items():
        results[finger] = transforms[contact_name][:3, 3].copy()

    return results


def get_all_finger_transforms(q_active):
    """Return dict[finger] -> 4x4 SE3 matrix in world coordinates."""
    sampled = get_sampled_contact_transforms(q_active)
    return {
        "index": sampled["index_tip_pad"],
        "middle": sampled["middle_tip_pad"],
        "ring": sampled["ring_tip_pad"],
        "little": sampled["little_tip_pad"],
        "thumb": sampled["thumb_tip_pad"],
    }


def get_sampled_contact_transforms(q_active, thumb_opp_mode=None):
    """Return dict[contact_name] -> 4x4 transform for all sampled finger and palm contact points."""
    q_f = (
        get_q_full(q_active)
        if thumb_opp_mode is None
        else _q_full_with_thumb_mode(q_active, thumb_opp_mode)
    )
    pin.forwardKinematics(model, data, q_f)

    results = {}
    for contact in CONTACT_DEFINITIONS:
        geom_info = COLLISION_GEOMETRIES[contact["geom"]]
        m_joint = data.oMi[geom_info["joint_id"]]
        m_geom_world = m_joint * geom_info["placement"]

        transform = m_geom_world.homogeneous.copy()
        transform[:3, 3] += m_geom_world.rotation @ contact["local_offset"]
        results[contact["name"]] = transform
    return results


def get_sampled_contact_dual_quaternions(q_active, thumb_opp_mode=None):
    transforms = get_sampled_contact_transforms(q_active, thumb_opp_mode=thumb_opp_mode)
    return {
        name: se3_matrix_to_dual_quaternion(tf)
        for name, tf in transforms.items()
    }


_align_contact_offsets_to_targets()


def _build_contact_table(samples, q_builder, contact_names, thumb_opp_mode=None):
    table = np.zeros((len(samples), len(contact_names), 8), dtype=np.float32)
    for i, value in enumerate(samples):
        q_active = q_builder(float(value))
        dq_map = get_sampled_contact_dual_quaternions(
            q_active,
            thumb_opp_mode=thumb_opp_mode,
        )
        for j, name in enumerate(contact_names):
            table[i, j] = dq_map[name]
    return table


def generate_contact_lut(resolution=11):
    """Generate dual-quaternion collision-contact LUT with finger and palm samples."""
    samples = np.linspace(0.0, 1.0, resolution)
    all_contacts = np.array(ALL_CONTACT_NAMES, dtype="<U64")

    index_names = np.array(CONTACT_NAMES_BY_GROUP["index"], dtype="<U64")
    mrl_names = np.array(
        CONTACT_NAMES_BY_GROUP["middle"]
        + CONTACT_NAMES_BY_GROUP["ring"]
        + CONTACT_NAMES_BY_GROUP["little"],
        dtype="<U64",
    )
    thumb_names = np.array(CONTACT_NAMES_BY_GROUP["thumb"], dtype="<U64")
    palm_names = np.array(CONTACT_NAMES_BY_GROUP["palm"], dtype="<U64")

    index_table = _build_contact_table(
        samples,
        q_builder=lambda s: np.array([0.0, s, 0.0], dtype=float),
        contact_names=index_names,
    )

    mrl_table = _build_contact_table(
        samples,
        q_builder=lambda s: np.array([0.0, 0.0, s], dtype=float),
        contact_names=mrl_names,
    )

    thumb_flex_table = _build_contact_table(
        samples,
        q_builder=lambda s: np.array([s, 0.0, 0.0], dtype=float),
        contact_names=thumb_names,
    )

    thumb_opp_mode0_table = _build_contact_table(
        samples,
        q_builder=lambda s: np.array([s, 0.0, 0.0], dtype=float),
        contact_names=all_contacts,
        thumb_opp_mode=THUMB_OPPOSITION_STATES[0],
    )

    thumb_opp_mode1_table = _build_contact_table(
        samples,
        q_builder=lambda s: np.array([s, 0.0, 0.0], dtype=float),
        contact_names=all_contacts,
        thumb_opp_mode=THUMB_OPPOSITION_STATES[1],
    )

    palm_neutral = get_sampled_contact_dual_quaternions(np.array([0.0, 0.0, 0.0], dtype=float))
    palm_table = np.vstack([palm_neutral[name] for name in palm_names]).astype(np.float32)

    joint_lower = np.array([JOINT_LIMITS[j][0] for j in JOINT_ORDER], dtype=np.float32)
    joint_upper = np.array([JOINT_LIMITS[j][1] for j in JOINT_ORDER], dtype=np.float32)
    contact_geometries = np.array([c["geom"] for c in CONTACT_DEFINITIONS], dtype="<U64")
    contact_groups = np.array([c["group"] for c in CONTACT_DEFINITIONS], dtype="<U16")
    contact_offsets = np.vstack([c["local_offset"] for c in CONTACT_DEFINITIONS]).astype(np.float32)

    np.savez(
        LUT_PATH,
        meta_version=np.array(["2.0"], dtype="<U8"),
        representation=np.array(["dual_quaternion_wxyz"], dtype="<U32"),
        resolution=np.array([resolution], dtype=np.int32),
        joint_names=np.array(JOINT_ORDER, dtype="<U32"),
        joint_limits_lower=joint_lower,
        joint_limits_upper=joint_upper,
        thumb_opposition_states=np.array(THUMB_OPPOSITION_STATES, dtype=np.float32),
        all_contact_names=all_contacts,
        all_contact_geometry_names=contact_geometries,
        all_contact_groups=contact_groups,
        all_contact_local_offsets=contact_offsets,
        index_contact_names=index_names,
        mrl_contact_names=mrl_names,
        thumb_contact_names=thumb_names,
        palm_contact_names=palm_names,
        index_table=index_table,
        mrl_table=mrl_table,
        thumb_flex_table=thumb_flex_table,
        thumb_opp_mode0_table=thumb_opp_mode0_table,
        thumb_opp_mode1_table=thumb_opp_mode1_table,
        palm_table=palm_table,
    )

    print(f"LUT saved to {LUT_PATH}")
    print(f"Resolution: {resolution}")
    print(f"Contacts per finger: index=4, middle=4, ring=4, little=4, thumb=4")
    print("Palm contacts: 6")
    print("Stored as dual quaternions with component order [qr_w, qr_x, qr_y, qr_z, qd_w, qd_x, qd_y, qd_z]")

    return {
        "index_table": index_table,
        "mrl_table": mrl_table,
        "thumb_flex_table": thumb_flex_table,
        "thumb_opp_mode0_table": thumb_opp_mode0_table,
        "thumb_opp_mode1_table": thumb_opp_mode1_table,
        "palm_table": palm_table,
    }


def generate_lut(resolution=11):
    """Compatibility wrapper to generate the contact LUT."""
    return generate_contact_lut(resolution=resolution)


if __name__ == "__main__":
    print("Collision geometries available for contact sampling:")
    for group_name, geom_names in CONTACT_GROUPS.items():
        print(f"  {group_name:<6}: {len(geom_names)} geometries")
        for geom_name in geom_names:
            g = COLLISION_GEOMETRIES[geom_name]
            print(f"    - {geom_name} ({g['type']})")

    print("\nRuntime joint limits used for LUT generation:")
    for j_name in JOINT_ORDER:
        lo, hi = JOINT_LIMITS[j_name]
        print(f"  {j_name}: [{lo:.6f}, {hi:.6f}]")

    import time

    start = time.time()
    for _ in range(1000):
        random_q = np.random.uniform(low, high)
        get_sampled_contact_transforms(random_q)
    end = time.time()
    print(f"\nTime for 1000 sampled-contact FK calls: {(end - start) * 1000:.4f} ms")

    print(f"\nRandom Config: {random_q}")
    contacts = get_sampled_contact_transforms(random_q)
    for c_name in [
        "index_tip_pad",
        "middle_tip_pad",
        "ring_tip_pad",
        "little_tip_pad",
        "thumb_tip_pad",
        "palm_distal_center",
    ]:
        pos = contacts[c_name][:3, 3]
        print(f"{c_name:<22} x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")

    print("\n" + "=" * 50)
    print("Generating Collision Contact LUT...")
    print("=" * 50)
    lut = generate_contact_lut(resolution=11)