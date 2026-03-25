import os
import numpy as np
import pinocchio as pin

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(SCRIPT_DIR, "mia_hand_description", "urdf", "mia_hand_flat.urdf")

# Active motor order: [Thumb_Flex, TISIT_Motor, MRL_Flex]
low = np.array([0.0, -1.0, 0.0], dtype=float)
high = np.array([1.0, 1.0, 1.00], dtype=float)

# Joint order in URDF: [j_index_fle, j_mrl_fle, j_thumb_fle, j_thumb_opp]
low_joint_limits = np.array([0, 0.0, 0.0, -0.628], dtype=float)
high_joint_limits = np.array([1.399999976, 1.396260023, 1.134500027, 0.0], dtype=float)


def _build_model_bundle(urdf_path):
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"Could not find URDF at: {urdf_path}")

    m = pin.buildModelFromUrdf(urdf_path)
    d = m.createData()
    g = pin.buildGeomFromUrdf(m, urdf_path, pin.GeometryType.COLLISION)
    return m, d, g


model, data, geom_model = _build_model_bundle(URDF_PATH)

# Mapping from finger name to collision geometry in the URDF.
tip_mapping = {
    "index": "mia_index_sensor_2",
    "middle": "mia_middle_sensor_2",
    "ring": "mia_ring_fle_2",
    "little": "mia_little_fle_2",
    "thumb": "mia_thumb_fle_2",
}


def _extract_tip_data(g_model, mapping):
    extracted = {}
    for finger, coll_name in mapping.items():
        if not g_model.existGeometryName(coll_name):
            raise RuntimeError(
                f"Could not find collision geometry '{coll_name}' for finger '{finger}'."
            )

        geom_id = g_model.getGeometryId(coll_name)
        geom_obj = g_model.geometryObjects[geom_id]
        radius = float(getattr(geom_obj.geometry, "radius", 0.0))

        extracted[finger] = {
            "joint_id": geom_obj.parentJoint,
            "placement": geom_obj.placement,
            "radius": radius,
        }
    return extracted


tip_data = _extract_tip_data(geom_model, tip_mapping)

def ctrl_to_joint(q_full):
    """Map from control range 0.0 to 1.0 to actual joint limits."""
    q_mapped = low_joint_limits + q_full * (high_joint_limits - low_joint_limits)
    return q_mapped

def get_q_full(q_active):
    """Map active motors [Thumb_Flex, TISIT_Motor, MRL_Flex] into full joint vector."""
    q_full = np.zeros(model.nq)

    # TISIT coupling logic.
    m2_pos = q_active[1]
    if m2_pos <= 0.0:
        index_val = -m2_pos
        opp_val = 0.0
    else:
        index_val = m2_pos
        opp_val = m2_pos

    # Convert to actual joint values based on limits.
    # [j_index_fle, j_mrl_fle, j_thumb_fle, j_thumb_opp]
    q_joint_vals = ctrl_to_joint([index_val, q_active[2], q_active[0], opp_val])

    q_full[model.getJointId("mia_j_thumb_fle") - 1] = q_joint_vals[2]

    q_full[model.getJointId("mia_j_index_fle") - 1] = q_joint_vals[0]
    q_full[model.getJointId("mia_j_thumb_opp") - 1] = q_joint_vals[3]

    q_full[model.getJointId("mia_j_mrl_fle") - 1] = q_joint_vals[1]
    q_full[model.getJointId("mia_j_ring_fle") - 1] = q_joint_vals[1]
    q_full[model.getJointId("mia_j_little_fle") - 1] = q_joint_vals[1]

    return q_full


def get_all_finger_positions(q_active):
    """Return dict[finger] -> np.array([x, y, z]) in world coordinates."""
    q_f = get_q_full(q_active)
    pin.forwardKinematics(model, data, q_f)

    results = {}
    for finger, info in tip_data.items():
        m_joint = data.oMi[info["joint_id"]]
        m_tip_world = m_joint * info["placement"]
        results[finger] = m_tip_world.translation.copy()

    return results


if __name__ == "__main__":
    for finger, coll_name in tip_mapping.items():
        radius = tip_data[finger]["radius"]
        print(f"Mapped {finger:<7} -> {coll_name} (r={radius:.4f}m)")

    import time
    start = time.time()
    for _ in range(1000):
        random_q = np.random.uniform(low, high)
        get_all_finger_positions(random_q)
    end = time.time()
    print(f"\nTime for 1000 forward kinematics calls: {(end - start) * 1000:.4f} ms")

    print(f"\nRandom Config: {random_q}")
    positions = get_all_finger_positions(random_q)
    for finger, pos in positions.items():
        radius = tip_data[finger]["radius"]
        print(
            f"{finger.capitalize():<7} Tip: x={pos[0]:.4f}, y={pos[1]:.4f}, "
            f"z={pos[2]:.4f} (r={radius:.4f})"
        )