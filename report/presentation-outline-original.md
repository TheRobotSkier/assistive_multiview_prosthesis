# Presentation Outline (Original)

## Grasp Preshaping

- Grasp preshaping problem
- Evolution of grasp pipeline from no iterations to sequential montecarlo to Evolutionary strategy
- The dimensionality realization around pose and rotation search space that made me propagate based on hitpoint compared to current pose
- The overall architecture of the pipeline (there is an image for this)

## ROI & TSDF

- How ROI prediction works, why it was necessary, and why it might not be as important currently
- How morton codes work (also nice existing figure)
- How we initially construct a TDF (no sign) using BFS

## Backside Estimation (Superquadric)

- The superquadric formula and how param change the shape
- How and why we compute an OBB for the initial estimate
- How gauss newton works
- How we construct a cost function from the superquadric formula
- Why we do thresholding to remove bad estimates from corrupting our TSDF
- How taubin distance works
- Why we do a newton raphson step (and some damping i think or maybe not)
- How we integrate the final estimate into the TSDF with domains of authority to get the final sign and why this was a challenge to do

## LUT

- How / why Pinocchio is nice
- How we flattened the URDF for use in this
- How we get the URDF collision primitives and how we compute contacts on these primitives
- Why we save to an NPZ and how we load it in Rust

## Evolution Strategy Optimization

- The dimensionality problem addressed
- Why it might be suitable for this project, vs overkill for how we ended up using it (only for wrist rotation since force controller could not accept different grasp types)
- What enhancements we made and why
- Maybe we should actually go more into the evolution here instead of at the beginning
- Also explain how we slowly moved from montecarlo to ES, and how the two are similar

## Evaluation

- Timing benchmark
- Latency and variability vs samples plot

## System Deployment

- How I run the system with Makefiles and the current debug setup
  - The containers, make run on the jetson and make run inside the prosthesis container
- The debug system: sysmon on host and jetson, rosbag recording, and analysis of logs and bags
- How jetson timesync works to get aligned timestamps on host
- How the UDP and large package splits mean that one package can invalidate a lot of packages, and why we run make network-tune
- How timestamping, DDS settings and best effort publishing works and why we use it
- Maybe show some of the debug plots I can now generate with the whole debug analysis

## Testing

- Test 1 should be explained how it was done, and maybe add the figure from the section
- Failure modes in test 3 is also very valuable to include, and maybe other plots from this section

## Future Work

- MobileSAM and TSDF fusion and how it might solve our current issues (not validated yet)
- GTSAM and FGO and why it also might help (not validated yet)