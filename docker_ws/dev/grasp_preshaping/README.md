# grasp_preshaping

This package provides the grasp preshaping solver core (Rust), the FFI type definitions shared between Rust and C++, and the ROS 2 service bridge node.

## Architecture Overview

### Data Loading

The pipeline starts by loading the point cloud and transform frames from ROS topics, which serve as inputs to the preshaping solver.

### Data Structure Construction

Internal data structures are pre-built to accelerate search and calculations:

1. **Lookup table** – Pre-generated for fast lookups
2. **Point cloud pruning** – Filter points to the region of interest
3. **TSDF construction** – Built from the pruned point cloud and camera positions:
   - Points are sorted using Morton codes for spatial locality
   - Signed distances are computed and then marked based on camera visibility to distinguish inside/outside surfaces

### Optimization Loop

The TSDF enables extremely fast distance and gradient queries. The optimization loop:

1. Searches across multiple perturbations of the hand state, propagated forward up to 5 seconds
2. Evaluates each perturbation by querying the TSDF at predicted fingertip positions
3. Scores grasps using type-specific cost functions (cylindrical, pinch, lateral)
4. Selects the best-scoring grasp as the output

## Future Work

**Wrist orientation handling** – Currently, the solver only searches across perturbations in hand position, time, and grasp type. Extending this to include wrist orientation would improve grasps. The approach could sample randomly across all dimensions or search each dimension separately.

**Trajectory simulation** – Adding trajectory simulation would help validate the overall workflow.

## Known Issues

- Rust clipping may be asymmetric
- Covariance is large for static objects