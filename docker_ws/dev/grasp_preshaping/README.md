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

This revised version incorporates the architectural shift in the code (removing the `Option` wrapper) and clarifies the importance of the **0.1** score for collisions to maintain a strong gradient signal.

---

## Grasp Scoring Improvements: Dense Reward Strategy

The goal of this refactor is to move from a **sparse** to a **dense reward landscape**. Currently, many failed grasps return no data, leaving the optimizer "blind" in large portions of the search space. By ensuring every state—even failures—returns a meaningful score, we provide a continuous gradient that guides the optimizer toward success rather than treating all non-successful grasps as equally discarded.

### 1. Implementation Architecture: Removing the `Option` Wrapper
To support a truly dense landscape, we must change how `score_grasp` is handled in `planner.rs`:
* **Remove the `Option<GraspScoreResult>` return type.** The function should now return a `GraspScoreResult` everywhere.
* **Avoid "Hard Fails":** Instead of returning `None` when a collision is detected or no contact is made, we instantiate a `GraspScoreResult` with low-tier scores (e.g., 0.1 or 0.0). This ensures the optimizer always has a struct to read from, preventing breaks in the gradient flow.

### 2. Tiered Scoring Model (The Contact Score)
The **Contact Score** will be our primary "unified quality/penalty metric." It captures whether the physical structure of the grasp makes sense. We will map failure modes to distinct score tiers to provide the optimizer with a "warmer/colder" signal:

* **Tier 1: Valid Grasp (Score: 0.8 to 1.0)**
    * *Criteria:* Hand swept closed, hit surface, good normals, and satisfies `min_fingers`.
    * *Meaning:* A successful grasp. Optimization here is about "polishing" (moving from good to great).

* **Tier 2: Soft Rejection (Score: 0.3 to 0.7)**
    * *Example:* Found collisions and good normals, but missing a finger constraint (`finger_set.len() < spec.min_fingers`).
    * *Actionable Fix:* Minor translation or rotation to catch the final finger.
    * *Calculation:* `base_contact_fraction * 0.5 (penalty multiplier)`.

* **Tier 3: Start Collision (Score: 0.1)**
    * *Example:* `sample == 0` collision (palm or fingers inside the object).
    * *Meaning:* The hand is "in" the object. 
    * *Actionable Fix:* Moderate translation (pulling back). 
    * **Crucial Detail:** We use **0.1** rather than a tiny epsilon (like `1e-6`) to provide a strong, non-negligible signal that tells the optimizer: *"You've found the object, but you're too deep. Back up!"*

* **Tier 4: No Collision (Score: 0.0)**
    * *Example:* Hand sweeps completely closed and hits nothing.
    * *Meaning:* Empty space; no physical data available.
    * *Actionable Fix:* Major translation needed to locate the object.

---

### 3. Metric Roles & Optimization Weights
While we have three core scores, they serve different purposes in the optimization loop:

1.  **Contact Score (Directional/Actionable):**
    * **Weight:** **High**.
    * **Role:** This provides the "gradient" that tells the hand how to move to reach a valid state. Because it transitions from 0.0 to 1.0 based on proximity and finger participation, it is the primary driver of convergence.

2.  **Alignment & Force Closure (Evaluative/Diagnostic):**
    * **Weight:** **Moderate**.
    * **Role:** These act as diagnostic signals. They tell us *why* a valid contact might still be a poor grasp (e.g., "the fingers are slipping sideways" or "the object will pop out"). These are crucial for differentiating a "good" grasp from a "great" one once Tier 1 is reached.

By weighting the **Contact Score** more heavily during the initial search, we ensure the planner prioritizes "finding and touching the object properly" before it begins obsessing over the perfect force-closure physics.

---

Your job is to map out the changes required and files that needs modification, verify your findings, and construct a detailed plan on how to do the refactor.


## Doing sampling

To do more efficient sampling, we should implement Sequential Monte Carlo (SMC) with decaying proposal variance. For this we need jitter_omega, jitter_v, elite_ratio, decay, and iterations. The current PREDICTION_SAMPLES should be used for each iteration. I propose starting values:

```rust
// Should be added to config.rs

// SMC Optimization Constants
pub const ITERATIONS: usize = 8; 
pub const DECAY_RATE: f64 = 0.75; // Geometric decay
pub const ELITE_RATIO: f64 = 0.1;

// Starting Proposal Variance (The "Wide Net")
pub const INITIAL_PROPOSAL_STD_V: f64 = 0.002;
pub const INITIAL_PROPOSAL_STD_OMEGA: f64 = 0.005;
```

I am not entirely sure what changes would need to happen for this to be implemnted succesfully, please investigate the files, and figure out exactly how to implment this, what files to modify, and verficaty your findings.

You should also examine how rayon or other optimisations can speed this search up, since we are doing alot of samples and they should be highly parallelizable. I am not sure if the current implementation is already doing this, but it is worth looking into.

## Integrations with trajecotry flow

I want to refine my publishing flow slightly in the preshping node. 

So straight away I want to publish the wrist rotations and some percentage of the calculated closure, where the percentage amount is defined in config.rs. The smaller closure amount shoud be sent straight to the controllers.

Then on some planner topics I want to publish the target hand pose from the planner, the full closure amount for each finger controller, and the grasp type. 

This way a trajectory node can subscribe to these topics, and then as we approch the target hand pose, we can start closing the hand fully. While this node already rotates the wrist and does a simple lighter closue to signify what the hand will do later on.

Take a look a this, and propose a detailed plan on how to implement this, what topics to publish, and how to integrate this with current bridge node. You do not need to worry about implementing this with the sim yet, just the ROS 2 side of things.

### 2nd iteration

I have a few things I want to change, namely:

/grasp_preshaping/target_finger_closures | Float64MultiArray │ Full [thumb, index, mrl] 0.0-1.0 closure amount 
/grasp_preshaping/target_hand_pose | geometry_msgs::msg::Pose | Full pose where the best grasp is predicted
/grasp_preshaping/wrist_pose | std_msgs/Float64 | Wrist rotations in degreed (0-360)

This means there is somthing with the postion where we need to get that from rust. keep the other definitions and topics, I did not mention.

Make a plan on how to implment this change, find the correct files and sections, and verify your findings.

## stuff

That sounds like some interesting findings. I have a few notes:

1. With grasps in tier, we might not have a massive truncations band. We can make these larger if you think, since it is a one time cost to make the tsdf, and not a huge part of the time equation. But it needs to make sense for makign the sampling more efficient.
2. In realtion to 1, I wonder if the first iteration mainly is concerned with location, and not grasp type, and that we might carry bad grasp types forward because of that, perhaps there should be some grasp type resampling as well, but it liekly should not be totally random, but maybe we can do some sort of weighted resampling based on the scores of the different grasp types, so that we are more likely to sample grasp types that are performing better, but still have some chance of sampling the others, to prevent getting stuck in local minima. 
    - As an extra note i wonder if we should allow more wrist rotations variance, or if the twist variance is good enough. What do you think? i am just nerveous that the inital sampling will not cover the space well wnough, and that we need to explore more with lower sample counts since it is a high demensional space.
3. Early termination is a good idea, i want the stopping tol in the config.rs. 
4. I think you r sugegstions on adjusting the perameters makes sense, but also i can not say that it will be a definete improvement.
5. You said: "When max contact_score < 0.1, increase probability weight to 2.0 (from 0.5)". Is this to promote more probable positions early on?

Could you go look into this and see if you perhaps want to refine your suggestions, or if you think there are other better paths for improvment on sampling effeciancy and overall speed? Please make sure to verify your findings, and then make a detailed plan on how to implement the changes you suggest, and what files and sections to modify.