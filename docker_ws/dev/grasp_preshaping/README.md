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

**Maybe better samples** - Not priority for now

## Potential Issues

- Rust clipping may be asymmetric
- Covariance is large for static objects
- Dealing with collisions in rest position
- Grasping is not based on finger positions but grasp types (maybe not a problem)
- The sign calculation and threshold. I have not tested this, since the sim have 2 cameras seeing from opposite sides.
- Self collisions are not currently handled, and do seem to be a problem
- Trajectory sim is not integrated with the planner for some reson, of maybe debug is just not working with it, atleast I don't get a debug dump.

- TSDF legend is black text and not very legeble, whuite would be better.
- Add tfs from ros2 (verify that hand is from ros and is correct)
- hand renders by default when changing to grasp mode
- Grasp should be a point and not sphere (and they are not sized, just the axis is sized)
- Only top 10 grasps should be shown with hand rest with only point and axis.

## TODO Debug Visualizer

- **TSDF legend visibility** – The legend fot the TSDF is currently rendered in black text on a black background, making it illegible. This is a simple rendering fix: switching to white text or a contrasting color would immediately improve visibility and usability of the debug visualizer.

- **ROS 2 transform verification** – Currently, it is unclear whether the hand transforms being visualized are sourced directly from ROS 2 topics or are hardcoded/inferred elsewhere. The intent is to verify that all transforms are properly imported from ROS 2 rather than relying on fallback or inferred values, ensuring the visualizer accurately represents the actual system state. this will require looking at how they are being recorded in the dump.

- **Hand visibility reset on mode switch** – When switching between hand display modes (shown vs. hidden), the hand state should reset so it does not render by default. Currently, switching grasp mode resets the hand mode to visible, requiring manual toggling. The expected behavior is that mode switches automatically reset the hand visibility state but to invisible.

- **Grasp representation** – Grasp points should be sized based on their score for visual ranking, but currently only the axis line markers show varying lengths. With potentially hundreds of grasps being rendered during optimization, switching to lightweight point representations would prevent the visualizer from freezing during large batches. The thing is that each call to render somethign or add something seem to be expensive, the less we 

- **Top 10 grasps filtering** – Currently, the visualizer may render all candidate grasps from the optimization loop, which can overwhelm the display when rendering 100+ hand poses. The solution is to filter and display only the top 10 highest-scoring grasps by their score values (which are already being computed), and only render the hand for these when in hand visible mode. This reduces visual clutter and improves the ability to evaluate the optimization results without performance degradation.

## TODO Grasping

- **Grasping is not based on finger positions but grasp types** – The current implementation optimizes over discrete grasp types (cylindrical, pinch, lateral) rather than directly optimizing finger positions. This is not necessarily a problem, but i have observed an issue, where the planner stops and thinks it found a solution, when only one finger touches an object, and the rest are in free space. This is not expected behavior, and it seems to me like the expected behavior would be that it gets a low score, since it is not a good grasp. I see 2 potential solutions to this. Fist is to fix the grasp scoring, and 2nd is to keep stepping the floating fingers until they also collide, and somehow normalize that to an appropriate grasp closure amount. My guess is that the first solution makes samping faster, but the 2nd might produce better grasps per sample, so i am not sure about which to choose for this tradeoff.

- **Wrist orientation handling** – Currently, the solver only searches across perturbations in hand position, time, and grasp type. Extending this to include wrist orientation would align better with the realworld hardware. The approach should sample randomly across all dimensions and not do per dimensions sampling, to get better spread and parrelalism. The program should return wrist orientation as part of the output, and the ROS node should publish it as well.

- **Asymetric clipping** - The Rust can currently do assymetric clipping, of the tsdf. This may be a good thing, but is want to understand if there are any issues with the current implementation. My original plan was to scale it to the roi, but then again if there is not data then why takup the whole roi? Examine the two options, and see if there are any implications of either.

- **Collisions in start position** - I have observed some issues where there are collisions in the start position, and it seems to me like the expected behavior would be to invalidate this starting position, and instead use ones that are not in collision.

- **Self collisions** - Self collisions are not currently handled, and do seem to be a problem. This is a tricky issue, since it is not just about checking if the hand collides with itself, but also how to handle it in the optimization loop. One approch would be to pre sample how far each finger can go in a given grasp type, and then use those ranges instead. This would deal with the difficulty, while remaining true to the real hand model. 

- **TSDF sign calculation** - I observed an issue where if 2 cameras are looking at the object from opposite sides, then the sign calculation can get messed up, and it seems to me like the expected behavior would be that we need to check if there are any occlusions from any of the cameras inrealtion to seeing the point. Since i belive we first make the TDF, i have a suspicions we can do some smarter calculations to get the sign, liek checking if any of the voxels along the ray from the camera to the point are occupied, and if they are then we can mark it as outside, and if they are not then we can mark it as inside. Then we would also need to deal with multiple cameras, but i think we already do that well. Tell me what you think about this approch.

- **Rayon** - (Not as important as other tasks.) Take a look at anything that could be parallelized with rayon, such as the perturbation search and evaluation.

## TODO Sim

- **Trajectory simulation** - There is a compose service that should simulate more realistic approch behavior, usually the grasp preshaping planner service would be run when the hand is in motion, and then imediatly rotate the wrist, and close the hand in the selected grasp type by some percentage configured in config.rs. Then when the hand is starting to approch the calcualted grasp position, it starts closing fully. My imidiate way to solve this would be to make some sort of cluster with a threshold also defined in config.rs where a soon as we approch grasp positions that are deemed succesful even in other wrist and grasp types, then we begin closing. You have to imagine a user moving the arm, and essentially being able to see the grasp starting to close, and being able to finetune the position where they close around the object. Tell me what you think of an approch like this, or if i am missing some obvious way to do it simpler. Another idea i had was to just have a distance threshold from selected grasp position defined in config.rs, and then as soon as we are within that distance, we start closing. Try also examine how to integrate theis into the sim. I imagine starting with the hand_trajectory_node would make sense.

## Maybe TODO
- **Large covariance** - The covariance can feel quite large if the hand is currently static, i am thinking we may need to somehow deal better with this. The thing is that it is not expected behavior to have the hand be static, but 

- **Grasps seems to get a score even when not touching** - In my debug vizualizer i see candidate grasps that are not touching the object, but still get a score, and it seems to me like the expected behavior would be that if a grasp is not touching the object, then it should get a very low score, since it is not a good grasp. This may be related to the issue with the grasp types, but it seems to me like we should also have some sort of penalty for grasps that are not touching the object, to prevent this from happening.

## Notes

Viz needs wrist integration