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

# Simplified geometry-centric contact definitions.
# Each contact specifies: name, group, geometry, and manual surface placement descriptor.
# Surface placements are resolved to local offsets via _resolve_surface_descriptor().
# 
# Surface types:
#   - "palmar": geometry-specific hand-facing surface
#       * cylinders: dorsal side in the current URDF frame
#       * spheres: dorsal side along the finger axis
#       * boxes: palm-facing side
#   - "lateral_pos": facing positive X (right side in local frame)
#   - "lateral_neg": facing negative X (left side in local frame)
#   - "distal": facing positive Z (tip/end direction)
#
# Contact naming convention:
#   Finger + Region + Side/Depth
# Examples: IndexPip, IndexPipSide, ThumbAddDip, ThumbAbdTip, PalmDistUlna.
#
# Contacts per geometry:
#   - Index (4 geoms): 2 each (top + side) = 8 total
#   - Middle (4 geoms): 1 each = 4 total
#   - Ring/Little/Thumb (3 geoms each): 1 each = 9 total
#   - Palm (2 geoms): 2 each (ulnar + radial) = 4 total
#   - TOTAL: 25 contacts

CONTACT_DEFINITIONS = [
    # Index: proximal flex geom (mia_index_fle_0)
    {"name": "IndexMcp", "group": "index", "geom": "mia_index_fle_0", "surface": "palmar"},
    {"name": "IndexMcpSide", "group": "index", "geom": "mia_index_fle_0", "surface": "lateral_pos"},
    
    # Index: proximal sensor geom (mia_index_sensor_0)
    {"name": "IndexDip", "group": "index", "geom": "mia_index_sensor_0", "surface": "palmar"},
    {"name": "IndexDipSide", "group": "index", "geom": "mia_index_sensor_0", "surface": "lateral_pos"},
    
    # Index: distal sensor geom (mia_index_sensor_1)
    {"name": "IndexPip", "group": "index", "geom": "mia_index_sensor_1", "surface": "palmar"},
    {"name": "IndexPipSide", "group": "index", "geom": "mia_index_sensor_1", "surface": "lateral_pos"},
    
    # Index: tip sensor geom (mia_index_sensor_2)
    {"name": "IndexTip", "group": "index", "geom": "mia_index_sensor_2", "surface": "palmar"},
    {"name": "IndexTipSide", "group": "index", "geom": "mia_index_sensor_2", "surface": "lateral_pos"},
    
    # Middle: one per geometry
    {"name": "MiddleMcp", "group": "middle", "geom": "mia_middle_fle_0", "surface": "palmar"},
    {"name": "MiddleDip", "group": "middle", "geom": "mia_middle_sensor_0", "surface": "palmar"},
    {"name": "MiddlePip", "group": "middle", "geom": "mia_middle_sensor_1", "surface": "palmar"},
    {"name": "MiddleTip", "group": "middle", "geom": "mia_middle_sensor_2", "surface": "palmar"},
    
    # Ring: one per geometry
    {"name": "RingDip", "group": "ring", "geom": "mia_ring_fle_0", "surface": "palmar"},
    {"name": "RingPip", "group": "ring", "geom": "mia_ring_fle_1", "surface": "palmar"},
    {"name": "RingTip", "group": "ring", "geom": "mia_ring_fle_2", "surface": "palmar"},
    
    # Little: one per geometry
    {"name": "LittleDip", "group": "little", "geom": "mia_little_fle_0", "surface": "palmar"},
    {"name": "LittlePip", "group": "little", "geom": "mia_little_fle_1", "surface": "palmar"},
    {"name": "LittleTip", "group": "little", "geom": "mia_little_fle_2", "surface": "palmar"},
    
    # Thumb: one per geometry
    {"name": "ThumbAddDip", "group": "thumb", "geom": "mia_thumb_fle_0", "surface": "lateral_neg"},
    {"name": "ThumbAddPip", "group": "thumb", "geom": "mia_thumb_fle_1", "surface": "lateral_neg"},
    {"name": "ThumbAddTip", "group": "thumb", "geom": "mia_thumb_fle_2", "surface": "lateral_neg"},
    
    # Palm: two per geometry (center and lateral support)
    {"name": "PalmProxUlna", "group": "palm", "geom": "mia_palm_0", "surface": "palm_ulnar"},
    {"name": "PalmProxRadi", "group": "palm", "geom": "mia_palm_0", "surface": "palm_radial"},
    {"name": "PalmDistUlna", "group": "palm", "geom": "mia_palm_1", "surface": "palm_ulnar"},
    {"name": "PalmDistRadi", "group": "palm", "geom": "mia_palm_1", "surface": "palm_radial"},
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


def _resolve_surface_descriptor(geom_info, surface_type, depth_scale=1.05):
    """
    Compute a local offset from a surface descriptor and geometry parameters.
    
    Args:
        geom_info: dict with keys 'type' and 'params', from COLLISION_GEOMETRIES
        surface_type: str describing target surface ("palmar", "lateral_pos", "lateral_neg", "distal", "side")
        depth_scale: multiplier for radius/half-extent to control offset magnitude (default 1.05 for proximal)
    
    Returns:
        np.array([x, y, z]) of local offset in the geometry's local frame
    
    Local frame conventions (consistent across all geometry types):
        - X: right (positive)
        - Y: proximal toward palm/base (negative Y is toward palm)
        - Z: distal toward apex/tip (positive Z is outward)
    """
    geom_type = geom_info["type"]
    params = geom_info["params"]

    if geom_type == "cylinder":
        r = params["radius"]
        l = params["length"]
        if surface_type == "palmar":
            return np.array([0.0, depth_scale * r, 0.0], dtype=float)
        elif surface_type == "lateral_pos":
            return np.array([depth_scale * r, -0.0, 0.0], dtype=float)
        elif surface_type == "lateral_neg":
            return np.array([-depth_scale * r, -0.0, 0.0], dtype=float)
        elif surface_type == "distal":
            return np.array([0.0, -depth_scale * r, 0.35 * l], dtype=float)
        else:
            raise RuntimeError(f"Unsupported surface '{surface_type}' for cylinder")

    elif geom_type == "sphere":
        r = params["radius"]
        if surface_type == "palmar":
            return np.array([0.0, 0.0, depth_scale * r], dtype=float)
        elif surface_type == "lateral_pos":
            return np.array([depth_scale * r, 0.0, 0.0], dtype=float)
        elif surface_type == "lateral_neg":
            return np.array([-depth_scale * r, 0.0, 0.0], dtype=float)
        elif surface_type == "distal":
            return np.array([0.0, -depth_scale * r, 0.2 * r], dtype=float)
        else:
            raise RuntimeError(f"Unsupported surface '{surface_type}' for sphere")

    elif geom_type == "box":
        hx, hy, hz = params["half_extents"]
        if surface_type == "palmar":
            return np.array([0.0, 0.0, depth_scale * hz], dtype=float)
        elif surface_type == "palm_radial":
            return np.array([depth_scale * 0.35 * hx, 0.0, depth_scale * hz], dtype=float)
        elif surface_type == "palm_ulnar":
            return np.array([-depth_scale * 0.35 * hx, 0.0, depth_scale * hz], dtype=float)
        else:
            raise RuntimeError(f"Unsupported surface '{surface_type}' for box")

    else:
        raise RuntimeError(f"Unknown geometry type '{geom_type}'")


# Initialize contact offsets from surface descriptors.
for contact in CONTACT_DEFINITIONS:
    geom_info = COLLISION_GEOMETRIES[contact["geom"]]
    surface_type = contact.get("surface", "palmar")
    depth_scale = contact.get("depth_scale", 1.05)
    contact["local_offset"] = _resolve_surface_descriptor(geom_info, surface_type, depth_scale)


# Note: Auto-alignment logic (_compute_palm_center_at_neutral, _compute_neutral_reference_points,
# _radial_candidates, and _align_contact_offsets_to_targets) has been removed.
# Contact offsets are now determined entirely by manual surface descriptors resolved via
# _resolve_surface_descriptor(), which provides deterministic and easily-editable placement.
# To adjust contact locations, edit the CONTACT_DEFINITIONS entries above or modify
# _resolve_surface_descriptor() to calibrate offset magnitudes.


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
        "index": "IndexTipTop",
        "middle": "MiddleTipTop",
        "ring": "RingTipTop",
        "little": "LittleTipTop",
        "thumb": "ThumbAddTip",
    }
    for finger, contact_name in primary.items():
        results[finger] = transforms[contact_name][:3, 3].copy()

    return results


def get_all_finger_transforms(q_active):
    """Return dict[finger] -> 4x4 SE3 matrix in world coordinates."""
    sampled = get_sampled_contact_transforms(q_active)
    return {
        "index": sampled["IndexTipTop"],
        "middle": sampled["MiddleTipTop"],
        "ring": sampled["RingTipTop"],
        "little": sampled["LittleTipTop"],
        "thumb": sampled["ThumbAddTip"],
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
    thumb_add_names = np.array(CONTACT_NAMES_BY_GROUP["thumb"], dtype="<U64")
    thumb_abd_names = np.array(
        [name.replace("ThumbAdd", "ThumbAbd", 1) for name in thumb_add_names],
        dtype="<U64",
    )
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

    thumb_opp_mode0_table = _build_contact_table(
        samples,
        q_builder=lambda s: np.array([s, 0.0, 0.0], dtype=float),
        contact_names=thumb_add_names,
        thumb_opp_mode=THUMB_OPPOSITION_STATES[0],
    )

    thumb_opp_mode1_table = _build_contact_table(
        samples,
        q_builder=lambda s: np.array([s, 0.0, 0.0], dtype=float),
        contact_names=thumb_add_names,
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
        meta_version=np.array(["3.0"], dtype="<U8"),
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
        thumb_add_contact_names=thumb_add_names,
        thumb_abd_contact_names=thumb_abd_names,
        palm_contact_names=palm_names,
        index_table=index_table,
        mrl_table=mrl_table,
        thumb_opp_mode0_table=thumb_opp_mode0_table,
        thumb_opp_mode1_table=thumb_opp_mode1_table,
        palm_table=palm_table,
    )

    print(f"LUT saved to {LUT_PATH}")
    print(f"Resolution: {resolution}")
    print(f"Contacts per finger: index=8 (2×4 geoms), middle=4, ring=3, little=3, thumb=3")
    print("Palm contacts: 4 (2×2 geoms)")
    print("Total contacts: 25")
    print("Stored as dual quaternions with component order [qr_w, qr_x, qr_y, qr_z, qd_w, qd_x, qd_y, qd_z]")

    return {
        "index_table": index_table,
        "mrl_table": mrl_table,
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

    print("\n" + "=" * 50)
    print("Generating Collision Contact LUT...")
    print("=" * 50)
    lut = generate_contact_lut(resolution=11)