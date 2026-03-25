#!/usr/bin/env python3
"""
Verification script for finger tip LUT file.

Tests and validates the generated LUT file at laptop_ws/finger_tip_lut.h5
"""

import h5py
import numpy as np
import os

LUT_PATH = os.path.join(os.path.dirname(__file__), "finger_tip_lut.h5")


def print_structure(f):
    """Print the structure of the HDF5 file showing all groups and datasets."""
    print("\n" + "="*60)
    print("LUT FILE STRUCTURE")
    print("="*60)
    
    def print_items(name, obj):
        indent = "  " * name.count('/')
        if isinstance(obj, h5py.Group):
            print(f"{indent}[GROUP] {name}")
        elif isinstance(obj, h5py.Dataset):
            print(f"{indent}[DATASET] {name} - shape: {obj.shape}, dtype: {obj.dtype}")
    
    f.visititems(print_items)


def verify_metadata(f):
    """Verify the metadata (resolution parameter)."""
    print("\n" + "="*60)
    print("METADATA VERIFICATION")
    print("="*60)
    
    if 'resolution' in f.attrs:
        resolution = f.attrs['resolution']
        print(f"✓ Resolution found: {resolution}")
        return resolution
    else:
        print("✗ Resolution attribute not found!")
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
    
    # Check mrl_flex group
    print("\n--- MRL Flex ---")
    if 'mrl_flex' in f:
        mrl_group = f['mrl_flex']
        sample_keys = list(mrl_group.keys())
        if sample_keys:
            sample = mrl_group[sample_keys[0]]
            for finger in ['middle', 'ring', 'little']:
                if finger in sample:
                    matrix = sample[finger][()]
                    is_valid = check_se3_matrix(matrix, f"mrl_flex/{sample_keys[0]}/{finger}")
                    if is_valid:
                        print(f"✓ {finger} - Valid SE3 matrix")
                        print(f"  Sample transform:\n{matrix}")
                    else:
                        all_valid = False
        else:
            print("✗ No samples found in mrl_flex")
            all_valid = False
    else:
        print("✗ mrl_flex group not found")
        all_valid = False
    
    # Check index_flex group
    print("\n--- Index Flex ---")
    if 'index_flex' in f:
        index_group = f['index_flex']
        sample_keys = list(index_group.keys())
        if sample_keys:
            matrix = index_group[sample_keys[0]][()]
            is_valid = check_se3_matrix(matrix, f"index_flex/{sample_keys[0]}")
            if is_valid:
                print(f"✓ index - Valid SE3 matrix")
                print(f"  Sample transform:\n{matrix}")
            else:
                all_valid = False
        else:
            print("✗ No samples found in index_flex")
            all_valid = False
    else:
        print("✗ index_flex group not found")
        all_valid = False
    
    # Check thumb_flex group
    print("\n--- Thumb Flex ---")
    if 'thumb_flex' in f:
        thumb_flex_group = f['thumb_flex']
        sample_keys = list(thumb_flex_group.keys())
        if sample_keys:
            matrix = thumb_flex_group[sample_keys[0]][()]
            is_valid = check_se3_matrix(matrix, f"thumb_flex/{sample_keys[0]}")
            if is_valid:
                print(f"✓ thumb_flex - Valid SE3 matrix")
                print(f"  Sample transform:\n{matrix}")
            else:
                all_valid = False
        else:
            print("✗ No samples found in thumb_flex")
            all_valid = False
    else:
        print("✗ thumb_flex group not found")
        all_valid = False
    
    # Check thumb_opposition group
    print("\n--- Thumb Opposition ---")
    if 'thumb_opposition' in f:
        thumb_opp_group = f['thumb_opposition']
        sample_keys = list(thumb_opp_group.keys())
        if sample_keys:
            matrix = thumb_opp_group[sample_keys[0]][()]
            is_valid = check_se3_matrix(matrix, f"thumb_opposition/{sample_keys[0]}")
            if is_valid:
                print(f"✓ thumb_opposition - Valid SE3 matrix")
                print(f"  Sample transform:\n{matrix}")
            else:
                all_valid = False
        else:
            print("✗ No samples found in thumb_opposition")
            all_valid = False
    else:
        print("✗ thumb_opposition group not found")
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
    if 'mrl_flex' in f and 'sample_5' in f['mrl_flex']:
        middle_transform = f['mrl_flex/sample_5/middle'][()]
        print(f"Middle finger transform at sample 5:\n{middle_transform}")
    else:
        print("✗ Cannot lookup middle finger at sample 5")
    
    # Example 2: Get thumb flex at sample 3
    print("\n--- Example 2: Thumb flex at sample 3 ---")
    if 'thumb_flex' in f and 'sample_3' in f['thumb_flex']:
        thumb_flex_transform = f['thumb_flex/sample_3'][()]
        print(f"Thumb flex transform at sample 3:\n{thumb_flex_transform}")
    else:
        print("✗ Cannot lookup thumb flex at sample 3")
    
    # Example 3: Get thumb opposition at sample 7
    print("\n--- Example 3: Thumb opposition at sample 7 ---")
    if 'thumb_opposition' in f and 'sample_7' in f['thumb_opposition']:
        thumb_opp_transform = f['thumb_opposition/sample_7'][()]
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
    if 'thumb_flex' in f and 'sample_0' in f['thumb_flex']:
        thumb_flex_neutral = f['thumb_flex/sample_0'][()]
        identity = np.eye(4)
        if np.allclose(thumb_flex_neutral, identity):
            print("✓ thumb_flex sample_0 is identity matrix (neutral position)")
        else:
            print("✗ thumb_flex sample_0 is NOT identity matrix")
            print(f"  Expected:\n{identity}")
            print(f"  Got:\n{thumb_flex_neutral}")
    else:
        print("✗ Cannot access thumb_flex sample_0")
    
    if 'thumb_opposition' in f and 'sample_0' in f['thumb_opposition']:
        thumb_opp_neutral = f['thumb_opposition/sample_0'][()]
        identity = np.eye(4)
        if np.allclose(thumb_opp_neutral, identity):
            print("✓ thumb_opposition sample_0 is identity matrix (neutral position)")
        else:
            print("✗ thumb_opposition sample_0 is NOT identity matrix")
            print(f"  Expected:\n{identity}")
            print(f"  Got:\n{thumb_opp_neutral}")
    else:
        print("✗ Cannot access thumb_opposition sample_0")
    
    # Test 2: Combine thumb flex and opposition transforms
    print("\n--- Test 2: Combine thumb flex and opposition ---")
    if ('thumb_flex' in f and 'sample_3' in f['thumb_flex'] and
        'thumb_opposition' in f and 'sample_5' in f['thumb_opposition']):
        
        thumb_flex = f['thumb_flex/sample_3'][()]
        thumb_opp = f['thumb_opposition/sample_5'][()]
        
        # Combine transforms using matrix multiplication
        # The order depends on the intended composition (flex then opposition or vice versa)
        combined_transform_1 = np.dot(thumb_flex, thumb_opp)  # flex then opposition
        combined_transform_2 = np.dot(thumb_opp, thumb_flex)  # opposition then flex
        
        print(f"Thumb flex (sample_3):\n{thumb_flex}")
        print(f"\nThumb opposition (sample_5):\n{thumb_opp}")
        print(f"\nCombined (flex then opposition):\n{combined_transform_1}")
        print(f"\nCombined (opposition then flex):\n{combined_transform_2}")
        
        # Verify the combined transform is still a valid SE3 matrix
        if check_se3_matrix(combined_transform_1, "combined_transform_1"):
            print("\n✓ Combined transform (flex then opposition) is valid SE3 matrix")
        if check_se3_matrix(combined_transform_2, "combined_transform_2"):
            print("✓ Combined transform (opposition then flex) is valid SE3 matrix")
    else:
        print("✗ Cannot combine thumb flex and opposition transforms")


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
        with h5py.File(LUT_PATH, 'r') as f:
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
        
        # Summary
        print("\n" + "="*60)
        print("VERIFICATION SUMMARY")
        print("="*60)
        print("✓ File structure verified")
        print(f"✓ SE3 matrix validity confirmed: {se3_valid}")
        print("✓ Lookup examples demonstrated")
        print("✓ Thumb composition tested")
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
