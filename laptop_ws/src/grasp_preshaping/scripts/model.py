import os
import numpy as np
import pinocchio as pin

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "urdf", "mia_hand_flat.urdf"))

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


def get_all_finger_transforms(q_active):
    """Return dict[finger] -> 4x4 SE3 matrix in world coordinates."""
    q_f = get_q_full(q_active)
    pin.forwardKinematics(model, data, q_f)

    results = {}
    for finger, info in tip_data.items():
        m_joint = data.oMi[info["joint_id"]]
        m_tip_world = m_joint * info["placement"]
        results[finger] = m_tip_world.homogeneous.copy()

    return results


def generate_lut(resolution=21):
    """
    Generate a lookup table (LUT) of SE3 matrices for different finger configurations.
    
    Args:
        resolution: Number of samples to generate across each actuation range (default: 21)
    
    Returns:
        Dictionary containing SE3 transforms for each finger configuration
    """
    lut = {}
    
    # Generate poses for MRL fingers (middle, ring, little)
    # All three MRL fingers move together (coupled)
    mrl_samples = np.linspace(0, 1, resolution)
    lut['mrl_flex'] = []
    for mrl_val in mrl_samples:
        q_active = np.array([0.0, 0.0, mrl_val])  # [thumb_flex=0, TISIT=0, mrl_flex]
        transforms = get_all_finger_transforms(q_active)
        lut['mrl_flex'].append({
            'middle': transforms['middle'],
            'ring': transforms['ring'],
            'little': transforms['little']
        })
    
    # Generate poses for index finger
    # Spread resolution samples across TISIT (0-1 only)
    index_samples = np.linspace(0, 1, resolution)
    lut['index_flex'] = []
    for tisit_val in index_samples:
        q_active = np.array([0.0, tisit_val, 0.0])  # [thumb_flex=0, TISIT, mrl_flex=0]
        transforms = get_all_finger_transforms(q_active)
        lut['index_flex'].append(transforms['index'])
    
    # Generate poses for thumb
    # Get neutral position transform (flex=0, opposition=0)
    q_neutral = np.array([0.0, 0.0, 0.0])
    transforms_neutral = get_all_finger_transforms(q_neutral)
    thumb_neutral = transforms_neutral['thumb']
    
    # a. thumb_flex: resolution samples across thumb flex (0-1), with opposition=0
    thumb_flex_samples = np.linspace(0, 1, resolution)
    lut['thumb_flex'] = []
    for flex_val in thumb_flex_samples:
        q_active = np.array([flex_val, 0.0, 0.0])  # [thumb_flex, TISIT=0, mrl_flex=0]
        transforms = get_all_finger_transforms(q_active)
        # Compute relative transform (not including base transform)
        # Multiply by inverse of neutral to get relative transform
        thumb_transform = transforms['thumb']
        thumb_relative = np.dot(thumb_transform, np.linalg.inv(thumb_neutral))
        lut['thumb_flex'].append(thumb_relative)
    
    # b. thumb_opposition: resolution samples across thumb opposition (0-1), with flex=0
    thumb_opp_samples = np.linspace(0, 1, resolution)
    lut['thumb_opposition'] = []
    for opp_val in thumb_opp_samples:
        # TISIT controls opposition when positive
        q_active = np.array([0.0, opp_val, 0.0])  # [thumb_flex=0, TISIT, mrl_flex=0]
        transforms = get_all_finger_transforms(q_active)
        # Compute relative transform (not including base transform)
        thumb_transform = transforms['thumb']
        thumb_relative = np.dot(thumb_transform, np.linalg.inv(thumb_neutral))
        lut['thumb_opposition'].append(thumb_relative)
    
    # Save LUT to a single file
    # Structure: dictionary with keys for each finger type
    # Each finger type has shape (resolution, 4, 4)
    output_path = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "data", "finger_tip_lut"))
    
    # Create a structured array or dictionary
    lut_data = {
        'resolution': resolution,
        'middle': np.array([entry['middle'] for entry in lut['mrl_flex']]),
        'ring': np.array([entry['ring'] for entry in lut['mrl_flex']]),
        'little': np.array([entry['little'] for entry in lut['mrl_flex']]),
        'index_flex': np.array(lut['index_flex']),
        'thumb_flex': np.array(lut['thumb_flex']),
        'thumb_opposition': np.array(lut['thumb_opposition']),
    }
    
    # Save as .npz (numpy's compressed format that can store multiple arrays)
    np.savez(output_path, **lut_data)
    
    print(f"LUT saved to {output_path}")
    print(f"Resolution: {resolution}")
    print(f"Entries:")
    print(f"  - mrl_flex: {len(lut['mrl_flex'])} samples (middle, ring, little)")
    print(f"  - index_flex: {len(lut['index_flex'])} samples")
    print(f"  - thumb_flex: {len(lut['thumb_flex'])} samples (relative)")
    print(f"  - thumb_opposition: {len(lut['thumb_opposition'])} samples (relative)")
    
    return lut


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
    
    # Generate LUT
    print("\n" + "="*50)
    print("Generating Finger Tip LUT...")
    print("="*50)
    lut = generate_lut(resolution=21)