# Marker Pose Covariance Plan

This is a planning document only. Do not treat it as implemented behavior.

## A. Goal

The marker node needs a realistic 6D measurement covariance for marker-derived
camera/IMU pose:

- translation covariance
- rotation covariance
- quality metrics that explain why a measurement is trusted or downweighted
- a path for feeding covariance into the external correction layer
- a path for later OpenVINS EKF marker updates

The covariance should describe marker measurement uncertainty, not pretend the
marker pose is perfect. The same concept should eventually feed both Phase 1
external correction and Phase 2 internal OpenVINS marker updates.

## B. Inputs Available From Marker Detector

Available or derivable inputs:

- marker side length in meters
- camera intrinsics `K`
- distortion coefficients
- detected 2D marker corners
- `solvePnP` `rvec` and `tvec`
- reprojection error in pixels
- marker image area and side length in pixels
- marker distance
- view angle / obliqueness
- number of valid markers
- agreement between multiple markers if visible
- temporal consistency over several frames
- OpenVINS odom covariance when marker measurements are used for correction
  gating

## C. Practical Covariance Estimation Approach

### Stage A: Heuristic Covariance

Start with a documented heuristic model:

- use configurable covariance floors
- increase covariance with marker distance
- increase covariance with reprojection error
- increase covariance when the marker covers few pixels
- increase covariance when viewed at a steep angle
- increase covariance when pose jumps frame-to-frame
- reject measurements before covariance weighting if quality is very poor

Hard rejection gates should run before covariance weighting. A low covariance
must not rescue a geometrically bad marker detection.

### Stage B: Empirical Calibration

Record bags with the marker at known poses, distances, and angles:

- compare marker-derived pose against static known geometry, or repeated
  stationary measurements
- estimate translation and rotation variance as a function of distance, marker
  area, reprojection error, and view angle
- use the result to tune heuristic coefficients, floors, and caps

This should be done with the actual D435i, printed markers, lighting, and
motion profiles used by the prosthesis system.

### Stage C: Analytical / Jacobian Approximation

If needed later, approximate pose covariance from `solvePnP` residual
sensitivity:

- assume corner pixel noise
- compute or approximate the Jacobian from corner residuals to pose parameters
- propagate pixel covariance into pose covariance
- compare the analytical estimate against empirical repeatability tests

This is more complex and should only be added if the heuristic and empirical
model are insufficient.

## D. Suggested Initial Covariance Model

A simple starting model:

- estimate pixel noise sigma from corner detection quality and reprojection
  error
- translation sigma scales roughly with marker distance and inverse marker
  pixel size
- rotation sigma scales with reprojection error and obliqueness
- use covariance floors so close markers do not become unrealistically perfect
- use covariance caps so bad measurements do not numerically dominate filters
- keep separate covariance terms for `x`, `y`, `z`, roll, pitch, and yaw

Example shape, not final truth:

```text
sigma_px = max(corner_noise_floor_px, reprojection_error_px)
size_px = sqrt(marker_area_px2)
range_factor = marker_distance_m / max(marker_side_length_m, small_value)

sigma_xy_m = floor_xy_m + k_xy * marker_distance_m * sigma_px / max(size_px, 1)
sigma_z_m  = floor_z_m  + k_z  * marker_distance_m * sigma_px / max(size_px, 1)
sigma_rot_rad = floor_rot_rad + k_rot * sigma_px / max(size_px, 1) + k_angle * obliqueness
```

Then clamp each term:

```text
sigma = min(max(sigma, floor), cap)
covariance = sigma^2
```

The model should be documented as an initial approximation and refined with
recorded data.

## E. How To Use Covariance In External Correction

Use this order:

1. Run hard gates first: known marker ID, duplicate ID rejection, area,
   reprojection error, distance, border/corner geometry, pose jump, and
   innovation gating.
2. Use covariance-weighted correction only after the measurement passes hard
   gates.
3. If VIO is healthy, apply small periodic corrections.
4. If VIO is invalid and marker quality is stable, allow hard reanchor with
   twist reset or twist covariance inflation.
5. Do not blindly trust a marker update just because its covariance is low if
   the geometry is poor.

The external correction should combine marker covariance, OpenVINS odom
covariance, and correction covariance when deciding whether and how strongly to
update `T_map_global`.

## F. How To Use Covariance In Future OpenVINS Internal EKF Update

For Phase 2, marker measurements should become proper EKF measurement updates:

- build `R_marker` from marker measurement covariance
- compute innovation in the EKF's pose/error-state representation
- use chi-square or Mahalanobis gating before accepting the update
- let the EKF update pose and relevant cross-covariances
- correct velocity or velocity cross-covariances only through a principled EKF
  update or explicit reset/reinitialize path supported by the estimator design
- avoid direct covariance hacking that makes the filter inconsistent

This work should start only after the external marker correction behavior is
validated and committed.

## G. Validation Plan

Validate covariance behavior with:

- stationary marker repeatability test
- distance sweep
- angle / obliqueness sweep
- motion blur test
- marker lost/regained test
- multiple marker agreement test
- RViz comparison of corrected odom covariance behavior during good VIO,
  bad VIO, marker-visible, and marker-lost segments
