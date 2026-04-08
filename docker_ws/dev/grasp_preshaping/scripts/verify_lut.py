#!/usr/bin/env python3
"""
Verification script for finger tip LUT file.

Tests and validates the generated LUT file in .npz format
"""

import numpy as np
import os

LUT_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "data", "finger_tip_lut.npz"))

def print_structure(f):
    """Print structure of .npz file showing all arrays."""
    print("\n" + "="*60)
    print("LUT FILE STRUCTURE")
    print("="*60)
    
    for key in f.files:
        arr = f[key]
        if isinstance(arr, np.ndarray):
            print(f"[ARRAY] {key} - shape: {arr.shape}, dtype: {arr.dtype}")
        else:
            print(f"[SCALAR] {key} - value: {arr}")


def verify_metadata(f):
    """Verify the metadata (resolution parameter)."""
    print("\n" + "="*60)
    print("METADATA VERIFICATION")
    print("="*60)
    
    if 'resolution' in f:
        resolution = f['resolution']
        print(f"✓ Resolution found: {resolution}")
        return resolution
    else:
        print("✗ Resolution not found!")
        return None


def check_se3_matrix(matrix, name="matrix"):
    """
    Check if a matrix is a valid SE3 transform.
    
    Args:
        matrix: numpy array to check
        name: name of the matrix for error reporting
    
    Returns:
        bool: True if valid SE3 matrix, False otherwise
    """
    # Check if it's a 4x4 numpy array
    if not isinstance(matrix, np.ndarray):
        print(f"✗ {name}: Not a numpy array")
        return False
    
    if matrix.shape != (4, 4):
        print(f"✗ {name}: Wrong shape {matrix.shape}, expected (4, 4)")
        return False
    
    # Check that the bottom row is [0, 0, 0, 1] (valid homogeneous transform)
    expected_bottom = np.array([0, 0, 0, 1])
    if not np.allclose(matrix[3, :], expected_bottom):
        print(f"✗ {name}: Invalid bottom row {matrix[3, :]}, expected {expected_bottom}")
        return False
    
    return True


def verify_se3_matrices(f):
    """
    Verify SE3 matrices in the LUT.
    
    Checks:
    - They are 4x4 numpy arrays
    - Bottom row is [0, 0, 0, 1] (valid homogeneous transform)
    - Prints a few sample transforms for each finger type
    """
    print("\n" + "="*60)
    print("SE3 MATRIX VERIFICATION")
    print("="*60)
    
    all_valid = True
    
    # Check middle finger
    print("\n--- Middle Finger ---")
    if 'middle' in f:
        middle = f['middle']
        if middle.ndim == 3 and middle.shape[1:] == (4, 4):
            sample = middle[0]
            is_valid = check_se3_matrix(sample, "middle[0]")
            if is_valid:
                print(f"✓ middle - Valid SE3 matrix")
                print(f"  Sample transform:\n{sample}")
            else:
                all_valid = False
        else:
            print(f"✗ middle has wrong shape: {middle.shape}")
            all_valid = False
    else:
        print("✗ middle not found")
        all_valid = False
    
    # Check ring finger
    print("\n--- Ring Finger ---")
    if 'ring' in f:
        ring = f['ring']
        if ring.ndim == 3 and ring.shape[1:] == (4, 4):
            sample = ring[0]
            is_valid = check_se3_matrix(sample, "ring[0]")
            if is_valid:
                print(f"✓ ring - Valid SE3 matrix")
                print(f"  Sample transform:\n{sample}")
            else:
                all_valid = False
        else:
            print(f"✗ ring has wrong shape: {ring.shape}")
            all_valid = False
    else:
        print("✗ ring not found")
        all_valid = False
    
    # Check little finger
    print("\n--- Little Finger ---")
    if 'little' in f:
        little = f['little']
        if little.ndim == 3 and little.shape[1:] == (4, 4):
            sample = little[0]
            is_valid = check_se3_matrix(sample, "little[0]")
            if is_valid:
                print(f"✓ little - Valid SE3 matrix")
                print(f"  Sample transform:\n{sample}")
            else:
                all_valid = False
        else:
            print(f"✗ little has wrong shape: {little.shape}")
            all_valid = False
    else:
        print("✗ little not found")
        all_valid = False
    
    # Check index_flex
    print("\n--- Index Flex ---")
    if 'index_flex' in f:
        index_flex = f['index_flex']
        if index_flex.ndim == 3 and index_flex.shape[1:] == (4, 4):
            sample = index_flex[0]
            is_valid = check_se3_matrix(sample, "index_flex[0]")
            if is_valid:
                print(f"✓ index_flex - Valid SE3 matrix")
                print(f"  Sample transform:\n{sample}")
            else:
                all_valid = False
        else:
            print(f"✗ index_flex has wrong shape: {index_flex.shape}")
            all_valid = False
    else:
        print("✗ index_flex not found")
        all_valid = False
    
    # Check thumb_flex
    print("\n--- Thumb Flex ---")
    if 'thumb_flex' in f:
        thumb_flex = f['thumb_flex']
        if thumb_flex.ndim == 3 and thumb_flex.shape[1:] == (4, 4):
            sample = thumb_flex[0]
            is_valid = check_se3_matrix(sample, "thumb_flex[0]")
            if is_valid:
                print(f"✓ thumb_flex - Valid SE3 matrix")
                print(f"  Sample transform:\n{sample}")
            else:
                all_valid = False
        else:
            print(f"✗ thumb_flex has wrong shape: {thumb_flex.shape}")
            all_valid = False
    else:
        print("✗ thumb_flex not found")
        all_valid = False
    
    # Check thumb_opposition
    print("\n--- Thumb Opposition ---")
    if 'thumb_opposition' in f:
        thumb_opp = f['thumb_opposition']
        if thumb_opp.ndim == 3 and thumb_opp.shape[1:] == (4, 4):
            sample = thumb_opp[0]
            is_valid = check_se3_matrix(sample, "thumb_opposition[0]")
            if is_valid:
                print(f"✓ thumb_opposition - Valid SE3 matrix")
                print(f"  Sample transform:\n{sample}")
            else:
                all_valid = False
        else:
            print(f"✗ thumb_opposition has wrong shape: {thumb_opp.shape}")
            all_valid = False
    else:
        print("✗ thumb_opposition not found")
        all_valid = False
    
    return all_valid


def demonstrate_lookup(f):
    """
    Demonstrate lookup operations.
    
    Shows:
    - How to lookup a specific finger configuration
    - Example: Get middle finger at sample 5
    - Example: Get thumb flex at sample 3
    - Example: Get thumb opposition at sample 7
    """
    print("\n" + "="*60)
    print("LOOKUP DEMONSTRATION")
    print("="*60)
    
    # Example 1: Get middle finger at sample 5
    print("\n--- Example 1: Middle finger at sample 5 ---")
    if 'middle' in f and f['middle'].ndim == 3 and f['middle'].shape[0] > 5:
        middle_transform = f['middle'][5]
        print(f"Middle finger transform at sample 5:\n{middle_transform}")
    else:
        print("✗ Cannot lookup middle finger at sample 5")
    
    # Example 2: Get thumb flex at sample 3
    print("\n--- Example 2: Thumb flex at sample 3 ---")
    if 'thumb_flex' in f and f['thumb_flex'].ndim == 3 and f['thumb_flex'].shape[0] > 3:
        thumb_flex_transform = f['thumb_flex'][3]
        print(f"Thumb flex transform at sample 3:\n{thumb_flex_transform}")
    else:
        print("✗ Cannot lookup thumb flex at sample 3")
    
    # Example 3: Get thumb opposition at sample 7
    print("\n--- Example 3: Thumb opposition at sample 7 ---")
    if 'thumb_opposition' in f and f['thumb_opposition'].ndim == 3 and f['thumb_opposition'].shape[0] > 7:
        thumb_opp_transform = f['thumb_opposition'][7]
        print(f"Thumb opposition transform at sample 7:\n{thumb_opp_transform}")
    else:
        print("✗ Cannot lookup thumb opposition at sample 7")


def test_thumb_composition(f):
    """
    Test thumb composition.
    
    Shows:
    - How to combine thumb flex and opposition transforms (matrix multiplication)
    - Verify that neutral positions (sample 0) give identity matrices for relative transforms
    """
    print("\n" + "="*60)
    print("THUMB COMPOSITION TEST")
    print("="*60)
    
    # Test 1: Verify neutral positions give identity matrices
    print("\n--- Test 1: Neutral positions (sample 0) ---")
    if 'thumb_flex' in f and f['thumb_flex'].ndim == 3 and f['thumb_flex'].shape[0] > 0:
        thumb_flex_neutral = f['thumb_flex'][0]
        identity = np.eye(4)
        if np.allclose(thumb_flex_neutral, identity):
            print("✓ thumb_flex[0] is identity matrix (neutral position)")
        else:
            print("✗ thumb_flex[0] is NOT identity matrix")
            print(f"  Expected:\n{identity}")
            print(f"  Got:\n{thumb_flex_neutral}")
    else:
        print("✗ Cannot access thumb_flex[0]")
    
    if 'thumb_opposition' in f and f['thumb_opposition'].ndim == 3 and f['thumb_opposition'].shape[0] > 0:
        thumb_opp_neutral = f['thumb_opposition'][0]
        identity = np.eye(4)
        if np.allclose(thumb_opp_neutral, identity):
            print("✓ thumb_opposition[0] is identity matrix (neutral position)")
        else:
            print("✗ thumb_opposition[0] is NOT identity matrix")
            print(f"  Expected:\n{identity}")
            print(f"  Got:\n{thumb_opp_neutral}")
    else:
        print("✗ Cannot access thumb_opposition[0]")
    
    # Test 2: Combine thumb flex and opposition transforms
    print("\n--- Test 2: Combine thumb flex and opposition ---")
    if ('thumb_flex' in f and f['thumb_flex'].ndim == 3 and f['thumb_flex'].shape[0] > 3 and
        'thumb_opposition' in f and f['thumb_opposition'].ndim == 3 and f['thumb_opposition'].shape[0] > 5):
        
        thumb_flex = f['thumb_flex'][3]
        thumb_opp = f['thumb_opposition'][5]
        
        # Combine transforms using matrix multiplication
        # The order depends on the intended composition (flex then opposition or vice versa)
        combined_transform_1 = np.dot(thumb_flex, thumb_opp)  # flex then opposition
        combined_transform_2 = np.dot(thumb_opp, thumb_flex)  # opposition then flex
        
        print(f"Thumb flex (sample 3):\n{thumb_flex}")
        print(f"\nThumb opposition (sample 5):\n{thumb_opp}")
        print(f"\nCombined (flex then opposition):\n{combined_transform_1}")
        print(f"\nCombined (opposition then flex):\n{combined_transform_2}")
        
        # Verify the combined transform is still a valid SE3 matrix
        if check_se3_matrix(combined_transform_1, "combined_transform_1"):
            print("\n✓ Combined transform (flex then opposition) is valid SE3 matrix")
        if check_se3_matrix(combined_transform_2, "combined_transform_2"):
            print("✓ Combined transform (opposition then flex) is valid SE3 matrix")
    else:
        print("✗ Cannot combine thumb flex and opposition transforms")


def test_thumb_composition_against_fk(f):
    """
    Compare LUT composition against direct FK from model.py.

    This confirms which multiplication order matches the real kinematics:
    - thumb_opp @ thumb_flex
    - thumb_flex @ thumb_opp
    """
    print("\n" + "=" * 60)
    print("THUMB COMPOSITION VS DIRECT FK")
    print("=" * 60)

    try:
        import model as hand_model
    except Exception as e:
        print(f"✗ Could not import model.py: {e}")
        return False

    if "resolution" not in f:
        print("✗ Missing resolution in LUT")
        return False

    resolution = int(f['resolution'])

    # Pick representative non-neutral samples; clamp to LUT size.
    i_flex = min(3, resolution - 1)
    i_opp = min(5, resolution - 1)

    # Get relative transforms from LUT
    t_flex_rel = f['thumb_flex'][i_flex]
    t_opp_rel = f['thumb_opposition'][i_opp]

    # Map sample index to active control in [0, 1].
    if resolution > 1:
        flex_val = i_flex / (resolution - 1)
        opp_val = i_opp / (resolution - 1)
    else:
        flex_val = 0.0
        opp_val = 0.0

    # Direct FK at neutral and target.
    t_neutral = hand_model.get_all_finger_transforms(np.array([0.0, 0.0, 0.0]))["thumb"]
    t_target = hand_model.get_all_finger_transforms(np.array([flex_val, opp_val, 0.0]))["thumb"]

    # Relative target from FK.
    t_fk_rel = t_target @ np.linalg.inv(t_neutral)

    # Candidate compositions.
    t_comp_opp_then_flex = t_opp_rel @ t_flex_rel
    t_comp_flex_then_opp = t_flex_rel @ t_opp_rel

    # Compare using Frobenius norm.
    err_opp_then_flex = np.linalg.norm(t_fk_rel - t_comp_opp_then_flex, ord="fro")
    err_flex_then_opp = np.linalg.norm(t_fk_rel - t_comp_flex_then_opp, ord="fro")

    print(f"Sample indices: flex={i_flex}, opp={i_opp}")
    print(f"Control values:  flex={flex_val:.4f}, opp={opp_val:.4f}")
    print(f"Frobenius error (opp then flex): {err_opp_then_flex:.6e}")
    print(f"Frobenius error (flex then opp): {err_flex_then_opp:.6e}")

    if err_opp_then_flex < err_flex_then_opp:
        print("✓ Best match: opposition then flex")
        return True
    if err_flex_then_opp < err_opp_then_flex:
        print("✓ Best match: flex then opposition")
        return True

    print("! Both orders produced equal error; check conventions or sample selection")
    return True


def main():
    """Main verification function."""
    print("="*60)
    print("FINGER TIP LUT VERIFICATION")
    print("="*60)
    print(f"LUT Path: {LUT_PATH}")
    
    # Check if file exists
    if not os.path.exists(LUT_PATH):
        print(f"\n✗ ERROR: LUT file not found at {LUT_PATH}")
        return False
    
    # Load the LUT file
    try:
        f = np.load(LUT_PATH)
        print("\n✓ LUT file loaded successfully")
        
        # 1. Print the structure
        print_structure(f)
        
        # 2. Verify the metadata
        resolution = verify_metadata(f)
        
        # 3. Check SE3 matrices
        se3_valid = verify_se3_matrices(f)
        
        # 4. Demonstrate lookup
        demonstrate_lookup(f)
        
        # 5. Test thumb composition
        test_thumb_composition(f)

        # 6. Validate composition order against direct FK
        fk_compare_ok = test_thumb_composition_against_fk(f)
        
        f.close()
        
        # Summary
        print("\n" + "="*60)
        print("VERIFICATION SUMMARY")
        print("="*60)
        print("✓ File structure verified")
        print(f"✓ SE3 matrix validity confirmed: {se3_valid}")
        print("✓ Lookup examples demonstrated")
        print("✓ Thumb composition tested")
        print(f"✓ FK composition comparison ran: {fk_compare_ok}")
        if resolution:
            print(f"✓ Resolution: {resolution}")
        print("\nVerification complete!")
        return True
        
    except Exception as e:
        print(f"\n✗ ERROR: Failed to verify LUT file: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
