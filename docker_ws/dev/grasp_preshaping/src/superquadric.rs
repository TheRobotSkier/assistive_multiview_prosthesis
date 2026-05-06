//! Superquadric-based backside estimation for TSDF shape completion.
//!
//! This module implements:
//! - Superquadric implicit function F(x,y,z) and its analytical gradient
//! - PCA-based orientation estimation
//! - Gauss-Newton fitting with locked rotation (6-DOF: scale + translation)
//! - Parallel template matching (Sphere, Box, Cylinder)
//! - Taubin distance approximation for SDF evaluation

use crate::config;
use nalgebra::{Matrix3, Matrix6, Rotation3, Vector3, Vector6};
use rayon::prelude::*;

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// Fitted superquadric parameters ready for SDF evaluation.
#[derive(Debug, Clone)]
pub struct SuperquadricParams {
    /// Shape exponents (fixed per template).
    pub epsilon1: f32,
    pub epsilon2: f32,
    /// Scale (half-extents) along local x, y, z axes.
    pub a: f32,
    pub b: f32,
    pub c: f32,
    /// Translation of the superquadric center in world frame.
    pub translation: Vector3<f32>,
    /// Rotation from world frame to superquadric local frame (locked from PCA).
    pub rotation: Rotation3<f32>,
    /// Which template won: 0 = sphere, 1 = box, 2 = cylinder.
    pub template_index: usize,
    /// Final fitting residual (mean squared error of F(p)-1 over all points).
    pub fit_error: f32,
}

/// A predefined shape template with fixed epsilon values.
#[derive(Debug, Clone, Copy)]
pub struct SuperquadricTemplate {
    pub epsilon1: f32,
    pub epsilon2: f32,
    pub name: &'static str,
}

/// The three priors used for parallel template matching.
pub const TEMPLATES: [SuperquadricTemplate; 3] = [
    SuperquadricTemplate {
        epsilon1: 1.0,
        epsilon2: 1.0,
        name: "sphere",
    },
    SuperquadricTemplate {
        epsilon1: 0.1,
        epsilon2: 0.1,
        name: "box",
    },
    SuperquadricTemplate {
        epsilon1: 0.1,
        epsilon2: 1.0,
        name: "cylinder",
    },
];

// ---------------------------------------------------------------------------
// Implicit function F(x,y,z) and gradient
// ---------------------------------------------------------------------------

/// Evaluate the superquadric implicit function in local frame coordinates.
///
/// The superquadric inside-outside function is:
///   F(x,y,z) = ( (|x/a|^(2/ε₁) + |y/b|^(2/ε₁) )^(ε₁/ε₂) + |z/c|^(2/ε₂) )^ε₂ - 1
///
/// Returns a value where:
/// - F = 0 on the surface
/// - F < 0 inside
/// - F > 0 outside
///
/// Uses the numerically stable formulation with `powf` to avoid issues
/// with very small epsilon values.
#[inline]
pub fn evaluate_f(
    lx: f32,
    ly: f32,
    lz: f32,
    a: f32,
    b: f32,
    c: f32,
    e1: f32,
    e2: f32,
) -> f32 {
    let xa = (lx / a).abs();
    let yb = (ly / b).abs();
    let zc = (lz / c).abs();

    // Clamp to avoid 0^negative issues
    let xa = xa.max(1e-10);
    let yb = yb.max(1e-10);
    let zc = zc.max(1e-10);

    let e1_inv = 1.0 / e1;
    let e2_inv = 1.0 / e2;

    let xy_term = xa.powf(2.0 * e1_inv) + yb.powf(2.0 * e1_inv);
    let xy_term = xy_term.max(1e-20);
    let xy_raised = xy_term.powf(e1 * e2_inv);

    let z_term = zc.powf(2.0 * e2_inv);

    let sum = xy_raised + z_term;
    let sum = sum.max(1e-20);

    sum.powf(e2) - 1.0
}

/// Evaluate the analytical gradient of F in local frame coordinates.
///
/// Returns (∂F/∂lx, ∂F/∂ly, ∂F/∂lz) — the gradient with respect to
/// local-frame position.
///
/// Derivation (verified symbolically):
///   Let u = (x/a)², v = (y/b)², w = (z/c)²
///   Let S_xy = u^(1/ε₁) + v^(1/ε₁)
///   Let S = S_xy^(ε₁/ε₂) + w^(1/ε₂)
///   Then F = S^ε₂ - 1
///
///   ∂F/∂x = ε₂ · S^(ε₂-1) · (ε₁/ε₂) · S_xy^(ε₁/ε₂ - 1) · (2/ε₁) · (x/a²) · u^(1/ε₁ - 1)
///          = S^(ε₂-1) · S_xy^(ε₁/ε₂ - 1) · (2x/a²) · u^(1/ε₁ - 1)
///
/// Similar for ∂F/∂y and ∂F/∂z.
#[inline]
pub fn evaluate_gradient_f(
    lx: f32,
    ly: f32,
    lz: f32,
    a: f32,
    b: f32,
    c: f32,
    e1: f32,
    e2: f32,
) -> Vector3<f32> {
    let xa = lx / a;
    let yb = ly / b;
    let zc = lz / c;

    let xa_abs = xa.abs().max(1e-10);
    let yb_abs = yb.abs().max(1e-10);
    let zc_abs = zc.abs().max(1e-10);

    let e1_inv = 1.0 / e1;
    let e2_inv = 1.0 / e2;

    // u = (x/a)^2, v = (y/b)^2, w = (z/c)^2
    let u = xa_abs * xa_abs;
    let v = yb_abs * yb_abs;
    let w = zc_abs * zc_abs;

    let u_safe = u.max(1e-20);
    let v_safe = v.max(1e-20);
    let w_safe = w.max(1e-20);

    // S_xy = u^(1/ε₁) + v^(1/ε₁)
    let u_pow = u_safe.powf(e1_inv);
    let v_pow = v_safe.powf(e1_inv);
    let s_xy = (u_pow + v_pow).max(1e-20);

    // w^(1/ε₂)
    let w_pow = w_safe.powf(e2_inv);

    // S = S_xy^(ε₁/ε₂) + w^(1/ε₂)
    let s_xy_raised = s_xy.powf(e1 * e2_inv);
    let s = (s_xy_raised + w_pow).max(1e-20);

    // Common factor: ε₂ · S^(ε₂-1)
    let s_factor = s.powf(e2 - 1.0);

    // ∂F/∂x = ε₂·S^(ε₂-1) · S_xy^(ε₁/ε₂-1) · u^(1/ε₁-1) · 2·xa/a
    //        = s_factor · s_xy_factor · u_factor · 2·xa/a
    let s_xy_factor = s_xy.powf(e1 * e2_inv - 1.0);

    let df_dx = if u > 1e-20 {
        let u_factor = u_safe.powf(e1_inv - 1.0);
        s_factor * s_xy_factor * u_factor * 2.0 * xa / a
    } else {
        0.0
    };

    let df_dy = if v > 1e-20 {
        let v_factor = v_safe.powf(e1_inv - 1.0);
        s_factor * s_xy_factor * v_factor * 2.0 * yb / b
    } else {
        0.0
    };

    // ∂F/∂z = S^(ε₂-1) · w^(1/ε₂-1) · 2·zc/c
    let df_dz = if w > 1e-20 {
        let w_factor = w_safe.powf(e2_inv - 1.0);
        s.powf(e2 - 1.0) * w_factor * 2.0 * zc / c
    } else {
        0.0
    };

    Vector3::new(df_dx, df_dy, df_dz)
}

impl SuperquadricParams {
    /// Transform a world-frame point into the superquadric local frame.
    #[inline]
    pub fn world_to_local(&self, world_pt: Vector3<f32>) -> Vector3<f32> {
        self.rotation * (world_pt - self.translation)
    }

    /// Evaluate F(x,y,z) at a world-frame point.
    #[inline]
    pub fn evaluate(&self, world_pt: Vector3<f32>) -> f32 {
        let local = self.world_to_local(world_pt);
        evaluate_f(
            local.x, local.y, local.z,
            self.a, self.b, self.c,
            self.epsilon1, self.epsilon2,
        )
    }

    /// Evaluate the gradient ∇F at a world-frame point (world-frame gradient).
    #[inline]
    pub fn gradient(&self, world_pt: Vector3<f32>) -> Vector3<f32> {
        let local = self.world_to_local(world_pt);
        let local_grad = evaluate_gradient_f(
            local.x, local.y, local.z,
            self.a, self.b, self.c,
            self.epsilon1, self.epsilon2,
        );
        // Chain rule: world gradient = R^T * local_gradient
        // Since rotation.transform() applies R, and we need R^T:
        self.rotation.inverse() * local_grad
    }

    /// Compute the Taubin distance approximation at a world-frame point.
    ///
    /// Returns a signed distance:
    /// - Negative inside the superquadric (F < 0)
    /// - Positive outside (F > 0)
    /// - Zero on the surface
    ///
    /// D ≈ F(x,y,z) / ||∇F(x,y,z)||
    /// (Using the signed version rather than |F-1| since we defined F = 0 at surface)
    pub fn taubin_distance(&self, world_pt: Vector3<f32>) -> f32 {
        let f_val = self.evaluate(world_pt);
        let grad = self.gradient(world_pt);
        let grad_norm = grad.norm();

        // Guard against near-zero gradient (interior of box-like shapes)
        if grad_norm < 1e-6 {
            // Fallback: use the raw function value as a distance proxy
            return f_val;
        }

        // Standard Taubin approximation: D = F / ||grad F||
        let d_taubin = f_val / grad_norm;

        // One Newton refinement step for improved accuracy.
        // Project the point along the gradient to the estimated surface,
        // then correct for the residual.
        let direction = grad / grad_norm;
        let projected_pt = world_pt - d_taubin * direction;
        let f_projected = self.evaluate(projected_pt);

        // If the projected point is very close to the surface, no correction needed
        if f_projected.abs() < 1e-6 {
            return d_taubin;
        }

        let grad_proj = self.gradient(projected_pt);
        let grad_norm_proj = grad_proj.norm();
        if grad_norm_proj < 1e-6 {
            return d_taubin;
        }

        d_taubin + f_projected / grad_norm_proj
    }

    /// Check if a world-frame point is inside the superquadric (F < 0).
    #[inline]
    pub fn is_inside(&self, world_pt: Vector3<f32>) -> bool {
        self.evaluate(world_pt) < 0.0
    }
}

// ---------------------------------------------------------------------------
// PCA and OBB initial guess
// ---------------------------------------------------------------------------

/// Result of PCA analysis on a point cloud.
pub struct PcaResult {
    pub centroid: Vector3<f32>,
    /// Eigenvectors as columns (rotation matrix from world to PCA frame).
    pub axes: Rotation3<f32>,
    /// Eigenvalues (variance along each axis), sorted descending.
    pub eigenvalues: Vector3<f32>,
}

/// Compute PCA on a set of 3D points.
///
/// Returns the centroid, principal axes (as a rotation), and eigenvalues.
/// The axes are sorted by descending eigenvalue (largest variance first).
pub fn compute_pca(points: &[Vector3<f32>]) -> PcaResult {
    assert!(!points.is_empty(), "cannot compute PCA on empty point set");

    // Centroid
    let n = points.len() as f32;
    let centroid: Vector3<f32> = points.iter().map(|p| *p).sum::<Vector3<f32>>() / n;

    // Covariance matrix (3x3, symmetric)
    let mut cov = Matrix3::zeros();
    for p in points {
        let d = p - centroid;
        cov += d * d.transpose();
    }
    cov /= n;

    // Eigendecomposition
    let eigen = cov.symmetric_eigen();

    // eigen.eigenvalues is sorted ascending by nalgebra.
    // eigen.eigenvectors columns correspond to eigenvalues.
    // We want descending order (largest first).
    let mut idx: [usize; 3] = [0, 1, 2];
    idx.sort_by(|&a, &b| {
        eigen.eigenvalues[b]
            .partial_cmp(&eigen.eigenvalues[a])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut sorted_eigenvalues = Vector3::zeros();
    let mut sorted_axes = Matrix3::zeros();
    for (out_col, &src_idx) in idx.iter().enumerate() {
        sorted_eigenvalues[out_col] = eigen.eigenvalues[src_idx];
        sorted_axes.set_column(out_col, &eigen.eigenvectors.column(src_idx));
    }

    // Ensure right-handed coordinate system
    if sorted_axes.determinant() < 0.0 {
        sorted_axes.set_column(2, &-sorted_axes.column(2));
    }

    PcaResult {
        centroid,
        axes: Rotation3::from_matrix_unchecked(sorted_axes),
        eigenvalues: sorted_eigenvalues,
    }
}

/// Compute the Oriented Bounding Box extents along PCA axes.
///
/// Returns (half_extents, center_in_world) — the initial guess for
/// superquadric scale (a, b, c) and translation.
pub fn compute_obb_extents(
    points: &[Vector3<f32>],
    pca: &PcaResult,
) -> (Vector3<f32>, Vector3<f32>) {
    // Project all points onto PCA axes
    let mut min_proj = Vector3::new(f32::MAX, f32::MAX, f32::MAX);
    let mut max_proj = Vector3::new(f32::MIN, f32::MIN, f32::MIN);

    for p in points {
        let local = pca.axes * (p - pca.centroid);
        min_proj.x = min_proj.x.min(local.x);
        min_proj.y = min_proj.y.min(local.y);
        min_proj.z = min_proj.z.min(local.z);
        max_proj.x = max_proj.x.max(local.x);
        max_proj.y = max_proj.y.max(local.y);
        max_proj.z = max_proj.z.max(local.z);
    }

    let half_extents = (max_proj - min_proj) * 0.5;
    let center_local = (min_proj + max_proj) * 0.5;
    let center_world = pca.centroid + pca.axes.inverse() * center_local;

    (half_extents, center_world)
}

// ---------------------------------------------------------------------------
// Gauss-Newton fitting (6-DOF: a, b, c, tx, ty, tz)
// ---------------------------------------------------------------------------

/// Pack the 6 optimizable parameters into a vector.
/// Order: [a, b, c, tx, ty, tz]
#[inline]
fn pack_params(a: f32, b: f32, c: f32, translation: Vector3<f32>) -> Vector6<f32> {
    Vector6::new(a, b, c, translation.x, translation.y, translation.z)
}

/// Unpack the 6 optimizable parameters.
#[inline]
fn unpack_params(params: &Vector6<f32>) -> (f32, f32, f32, Vector3<f32>) {
    (
        params[0].abs().max(0.001),
        params[1].abs().max(0.001),
        params[2].abs().max(0.001),
        Vector3::new(params[3], params[4], params[5]),
    )
}

/// Compute the analytical Jacobian of F w.r.t. the 6 parameters
/// for a single point.
///
/// Parameters: [a, b, c, tx, ty, tz]
///
/// The Jacobian entries are derived using the chain rule:
///   ∂F/∂a = ∂F/∂(lx) · ∂(lx)/∂a + direct ∂F/∂a
///   where lx = R·(p - t) · e_x  and a appears in the implicit function
///
/// More precisely, for parameter a:
///   lx = R·(p-t) · e_x  (does not depend on a)
///   ∂F/∂a = ∂F/∂(lx/a) · ∂(lx/a)/∂a + ... 
///   = -(2·lx²/a³)^(1/ε₁) · (ε₁/ε₂) · S_xy^(ε₁/ε₂ - 1) · ε₂ · S^(ε₂-1) / (lx²/a²)^(1-1/ε₁)
///   
/// We use the simpler formulation: compute ∂F/∂a via finite-difference-like
/// analytical derivatives. Since we already have ∂F/∂lx, ∂F/∂ly, ∂F/∂lz,
/// we can use:
///   ∂F/∂tx = -(∂F/∂lx · R₁₁ + ∂F/∂ly · R₂₁ + ∂F/∂lz · R₃₁)
///   ∂F/∂ty = -(∂F/∂lx · R₁₂ + ∂F/∂ly · R₂₂ + ∂F/∂lz · R₃₂)
///   ∂F/∂tz = -(∂F/∂lx · R₁₃ + ∂F/∂ly · R₂₃ + ∂F/∂lz · R₃₃)
///
/// For scale parameters, we use the chain rule through the implicit function:
///   ∂F/∂a = ∂F/∂(lx/a) · (-lx/a²)  (but this is complex)
///
/// Instead, we compute the full analytical Jacobian directly.
fn compute_jacobian_row(
    world_pt: Vector3<f32>,
    params: &Vector6<f32>,
    e1: f32,
    e2: f32,
    rotation: &Rotation3<f32>,
) -> Vector6<f32> {
    let (a, b, c, translation) = unpack_params(params);

    // Local coordinates
    let d = world_pt - translation;
    let local = rotation * d;
    let lx = local.x;
    let ly = local.y;
    let lz = local.z;

    // Evaluate F and its spatial gradient at this point
    let grad_local = evaluate_gradient_f(lx, ly, lz, a, b, c, e1, e2);

    // Translation derivatives: ∂F/∂t = -R^T · ∇_local F
    // (Because moving the center by +dt moves the point in local frame by -R·dt)
    let grad_world = rotation.inverse() * grad_local;
    let df_dtx = -grad_world.x;
    let df_dty = -grad_world.y;
    let df_dtz = -grad_world.z;

    // Scale derivatives via chain rule through the implicit function.
    // For parameter a: F depends on a through (lx/a) terms.
    // ∂F/∂a = -lx/a · (∂F/∂lx) / a = -(lx/a²) · ∂F/∂(lx/a)
    // Since ∂F/∂lx = (∂F/∂(lx/a)) · (1/a), we get:
    // ∂F/∂a = -lx · (∂F/∂lx) / a
    let df_da = -lx * grad_local.x / a;
    let df_db = -ly * grad_local.y / b;
    let df_dc = -lz * grad_local.z / c;

    Vector6::new(df_da, df_db, df_dc, df_dtx, df_dty, df_dtz)
}

/// Fit a single superquadric template to the point cloud.
///
/// Uses Gauss-Newton with Levenberg-Marquardt damping on the 6 parameters
/// (a, b, c, tx, ty, tz). Rotation is locked from PCA.
///
/// Returns the fitted parameters and the mean squared fitting error.
pub fn fit_superquadric_template(
    points: &[Vector3<f32>],
    template: SuperquadricTemplate,
    initial_a: f32,
    initial_b: f32,
    initial_c: f32,
    initial_translation: Vector3<f32>,
    rotation: &Rotation3<f32>,
) -> (SuperquadricParams, f32) {
    let e1 = template.epsilon1;
    let e2 = template.epsilon2;
    let max_iter = config::SQ_MAX_GN_ITERATIONS;
    let damping = config::SQ_GN_DAMPING;

    let mut params = pack_params(initial_a, initial_b, initial_c, initial_translation);

    let n_points = points.len();
    let mut best_error = f32::MAX;
    let mut best_params = params;

    for _iter in 0..max_iter {
        // Accumulate JᵀJ and Jᵀr
        let mut hessian = Matrix6::zeros();
        let mut rhs = Vector6::zeros();
        let mut total_error = 0.0f32;

        for pt in points {
            let (a, b, c, translation) = unpack_params(&params);
            let local = rotation * (pt - translation);
            let residual = evaluate_f(local.x, local.y, local.z, a, b, c, e1, e2);

            total_error += residual * residual;

            let j_row = compute_jacobian_row(*pt, &params, e1, e2, rotation);

            // JᵀJ += j_rowᵀ · j_row (outer product)
            hessian += j_row * j_row.transpose();
            // Jᵀr += j_rowᵀ · residual
            rhs += j_row * residual;
        }

        let mse = total_error / n_points as f32;

        // Track best solution
        if mse < best_error {
            best_error = mse;
            best_params = params;
        }

        // Levenberg-Marquardt: add damping to diagonal
        for i in 0..6 {
            hessian[(i, i)] += damping * hessian[(i, i)].max(1e-10);
        }

        // Solve (JᵀJ + λI) · δ = -Jᵀr
        let neg_rhs = -rhs;
        if let Some(delta) = hessian.lu().solve(&neg_rhs) {
            // Clamp step size to prevent wild jumps
            let step_norm: f32 = delta.norm();
            let max_step = 0.05; // 5cm max step per iteration
            let scaled_delta = if step_norm > max_step {
                delta * (max_step / step_norm)
            } else {
                delta
            };
            params += scaled_delta;

            // Ensure scale parameters stay positive
            params[0] = params[0].max(0.001);
            params[1] = params[1].max(0.001);
            params[2] = params[2].max(0.001);
        } else {
            // Solver failed — stop iterating
            break;
        }
    }

    // Final error evaluation: compare best_params (tracked during iterations)
    // against final params (the last iteration's result).
    let (best_a, best_b, best_c, best_t) = unpack_params(&best_params);
    let mut best_error_recheck = 0.0f32;
    for pt in points {
        let local = rotation * (pt - best_t);
        let r = evaluate_f(local.x, local.y, local.z, best_a, best_b, best_c, e1, e2);
        best_error_recheck += r * r;
    }
    best_error_recheck /= n_points as f32;

    let (a, b, c, translation) = unpack_params(&params);
    let final_mse = {
        let mut e = 0.0f32;
        for pt in points {
            let local = rotation * (pt - translation);
            let r = evaluate_f(local.x, local.y, local.z, a, b, c, e1, e2);
            e += r * r;
        }
        e / n_points as f32
    };
    // Use whichever is better
    let (use_a, use_b, use_c, use_t) = if final_mse <= best_error_recheck {
        (a, b, c, translation)
    } else {
        (best_a, best_b, best_c, best_t)
    };
    let use_error = final_mse.min(best_error_recheck);

    let result = SuperquadricParams {
        epsilon1: e1,
        epsilon2: e2,
        a: use_a,
        b: use_b,
        c: use_c,
        translation: use_t,
        rotation: *rotation,
        template_index: match template.name {
            "sphere" => 0,
            "box" => 1,
            "cylinder" => 2,
            _ => 0,
        },
        fit_error: use_error,
    };

    (result, use_error)
}

// ---------------------------------------------------------------------------
// Parallel template matching (top-level entry point)
// ---------------------------------------------------------------------------

/// Fit the best superquadric to a point cloud using parallel template matching.
///
/// This is the main entry point for backside estimation:
/// 1. Compute PCA to lock the rotation
/// 2. Compute OBB extents for initial scale/translation guess
/// 3. Run Gauss-Newton for each template in parallel (rayon)
/// 4. Return the template with the lowest fitting error
///
/// Returns `None` if the point cloud is too small or fitting fails.
pub fn fit_best_superquadric(points: &[Vector3<f32>]) -> Option<SuperquadricParams> {
    if points.len() < config::SQ_MIN_FIT_POINTS {
        return None;
    }

    // Step 1: PCA for orientation
    let pca = compute_pca(points);

    // Step 2: OBB for initial guess
    let (half_extents, center) = compute_obb_extents(points, &pca);

    // Ensure minimum scale
    let initial_a = half_extents.x.max(0.005);
    let initial_b = half_extents.y.max(0.005);
    let initial_c = half_extents.z.max(0.005);

    // Step 3: Parallel template matching
    let results: Vec<(SuperquadricParams, f32)> = TEMPLATES
        .par_iter()
        .map(|template| {
            fit_superquadric_template(
                points,
                *template,
                initial_a,
                initial_b,
                initial_c,
                center,
                &pca.axes,
            )
        })
        .collect();

    // Step 4: Select the best template
    let (best_params, best_error) = results
        .into_iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))?;

    // If the fitting error is too high, skip backside estimation
    if best_error > config::SQ_FIT_ERROR_THRESHOLD {
        return None;
    }

    Some(best_params)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn test_sphere_f_at_surface() {
        // Unit sphere: ε₁=1, ε₂=1, a=b=c=1
        let e1 = 1.0;
        let e2 = 1.0;
        // Points on the surface of a unit sphere should give F ≈ 0
        let surface_points = [
            (1.0_f32, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.7071, 0.7071, 0.0),
        ];
        for (x, y, z) in &surface_points {
            let f = evaluate_f(*x, *y, *z, 1.0, 1.0, 1.0, e1, e2);
            assert!(
                f.abs() < 0.01,
                "F({:.3},{:.3},{:.3}) = {:.6}, expected ≈ 0",
                x, y, z, f
            );
        }
    }

    #[test]
    fn test_sphere_f_inside_outside() {
        let e1 = 1.0;
        let e2 = 1.0;
        // Inside point
        let f_in = evaluate_f(0.5, 0.0, 0.0, 1.0, 1.0, 1.0, e1, e2);
        assert!(f_in < 0.0, "inside point should have F < 0, got {}", f_in);

        // Outside point
        let f_out = evaluate_f(2.0, 0.0, 0.0, 1.0, 1.0, 1.0, e1, e2);
        assert!(
            f_out > 0.0,
            "outside point should have F > 0, got {}",
            f_out
        );
    }

    #[test]
    fn test_box_f_at_surface() {
        // Box: ε₁=0.1, ε₂=0.1, a=b=c=1
        let e1 = 0.1;
        let e2 = 0.1;
        // Points on the face of the box
        let f = evaluate_f(1.0, 0.0, 0.0, 1.0, 1.0, 1.0, e1, e2);
        assert!(
            f.abs() < 0.05,
            "F at box surface = {:.6}, expected ≈ 0",
            f
        );
    }

    #[test]
    fn test_gradient_at_surface_points_sphere() {
        // For a unit sphere, gradient at (1,0,0) should point in +x direction
        let grad = evaluate_gradient_f(1.0, 0.01, 0.01, 1.0, 1.0, 1.0, 1.0, 1.0);
        assert!(
            grad.x > 0.5,
            "gradient at (1,0,0) should point +x, got {:?}",
            grad
        );
    }

    #[test]
    fn test_taubin_distance_sphere_surface() {
        let sq = SuperquadricParams {
            epsilon1: 1.0,
            epsilon2: 1.0,
            a: 1.0,
            b: 1.0,
            c: 1.0,
            translation: Vector3::zeros(),
            rotation: Rotation3::identity(),
            template_index: 0,
            fit_error: 0.0,
        };
        // At the surface, Taubin distance should be ≈ 0
        let d = sq.taubin_distance(Vector3::new(1.0, 0.0, 0.0));
        assert!(
            d.abs() < 0.1,
            "Taubin distance at sphere surface = {:.6}, expected ≈ 0",
            d
        );

        // Outside point at distance ~1 from surface
        let d_out = sq.taubin_distance(Vector3::new(2.0, 0.0, 0.0));
        assert!(
            d_out > 0.5,
            "Taubin distance outside should be > 0.5, got {:.6}",
            d_out
        );

        // Inside point
        let d_in = sq.taubin_distance(Vector3::new(0.0, 0.0, 0.0));
        assert!(
            d_in < -0.1,
            "Taubin distance inside should be < -0.1, got {:.6}",
            d_in
        );
    }

    #[test]
    fn test_pca_on_line_points() {
        // Points along the x-axis
        let points: Vec<Vector3<f32>> = (0..100)
            .map(|i| Vector3::new(i as f32 * 0.01, 0.0, 0.0))
            .collect();

        let pca = compute_pca(&points);

        // First principal component should be along x
        let first_axis = pca.axes * Vector3::x();
        let dot_x = first_axis.x.abs();
        assert!(
            dot_x > 0.9,
            "first PCA axis should align with x, dot = {:.4}",
            dot_x
        );

        // Largest eigenvalue should be along x
        assert!(
            pca.eigenvalues[0] > pca.eigenvalues[1],
            "eigenvalues not sorted descending"
        );
    }

    #[test]
    fn test_pca_on_plane_points() {
        // Points on the xy-plane
        let points: Vec<Vector3<f32>> = (0..50)
            .flat_map(|i| (0..50).map(move |j| Vector3::new(i as f32 * 0.01, j as f32 * 0.01, 0.0)))
            .collect();

        let pca = compute_pca(&points);

        // Smallest eigenvalue should be near zero (z direction)
        assert!(
            pca.eigenvalues[2] < pca.eigenvalues[0] * 0.01,
            "smallest eigenvalue should be near zero for planar points, got {:?}",
            pca.eigenvalues
        );
    }

    #[test]
    fn test_obb_extents() {
        // Dense points on the faces of a box from (0,0,0) to (2,4,6).
        // Many points are needed so PCA reliably recovers the box axes.
        let mut points = Vec::new();
        for i in 0..10 {
            for j in 0..10 {
                let u = i as f32 * 0.2;
                let v = j as f32 * 0.4;
                // +z face
                points.push(Vector3::new(u, v, 6.0));
                // -z face
                points.push(Vector3::new(u, v, 0.0));
                // +y face
                points.push(Vector3::new(u, 4.0, v * 6.0 / 4.0));
                // -y face
                points.push(Vector3::new(u, 0.0, v * 6.0 / 4.0));
            }
        }

        let pca = compute_pca(&points);
        let (half_extents, _center) = compute_obb_extents(&points, &pca);

        // Half-extents should be approximately [1, 2, 3] (sorted by PCA)
        // PCA sorts by variance, so the largest extent (z=6) should have the largest half-extent.
        let mut extents = [half_extents.x, half_extents.y, half_extents.z];
        extents.sort_by(|a, b| b.partial_cmp(a).unwrap());
        assert!(
            (extents[0] - 3.0).abs() < 0.5,
            "largest half extent = {:.3}, expected ~3.0",
            extents[0]
        );
        assert!(
            (extents[1] - 2.0).abs() < 0.5,
            "middle half extent = {:.3}, expected ~2.0",
            extents[1]
        );
        assert!(
            (extents[2] - 1.0).abs() < 0.5,
            "smallest half extent = {:.3}, expected ~1.0",
            extents[2]
        );
    }

    #[test]
    fn test_fit_sphere_points() {
        // Generate points on the front half of a sphere (simulating partial observation)
        let center = Vector3::new(0.0, 0.0, 0.0);
        let radius = 0.03_f32; // 3cm
        let mut points = Vec::new();
        for i in 0..200 {
            let theta = std::f32::consts::PI * (i as f32 / 200.0); // 0 to PI (front half)
            for j in 0..20 {
                let phi = 2.0 * std::f32::consts::PI * (j as f32 / 20.0);
                let x = center.x + radius * theta.sin() * phi.cos();
                let y = center.y + radius * theta.sin() * phi.sin();
                let z = center.z + radius * theta.cos();
                points.push(Vector3::new(x, y, z));
            }
        }

        let result = fit_best_superquadric(&points);
        assert!(result.is_some(), "fitting should succeed for sphere points");

        let sq = result.unwrap();
        // The fitted scales should be close to the sphere radius
        let avg_scale = (sq.a + sq.b + sq.c) / 3.0;
        assert!(
            (avg_scale - radius).abs() < radius * 0.5,
            "avg scale = {:.4}, expected ~{:.4}",
            avg_scale,
            radius
        );
    }

    #[test]
    fn test_fit_returns_none_for_few_points() {
        let points = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let result = fit_best_superquadric(&points);
        assert!(
            result.is_none(),
            "should return None for < SQ_MIN_FIT_POINTS"
        );
    }

    #[test]
    fn test_world_to_local_roundtrip() {
        let sq = SuperquadricParams {
            epsilon1: 1.0,
            epsilon2: 1.0,
            a: 1.0,
            b: 2.0,
            c: 3.0,
            translation: Vector3::new(1.0, 2.0, 3.0),
            rotation: Rotation3::from_axis_angle(&Vector3::z_axis(), 0.5),
            template_index: 0,
            fit_error: 0.0,
        };

        let world_pt = Vector3::new(1.5, 2.5, 3.5);
        let local = sq.world_to_local(world_pt);
        // Transform back: world = R * local + translation
        let recovered = sq.rotation.inverse() * local + sq.translation;
        assert!(
            (recovered - world_pt).norm() < 1e-5,
            "roundtrip failed: {:?} vs {:?}",
            recovered,
            world_pt
        );
    }

    #[test]
    fn test_is_inside_outside() {
        let sq = SuperquadricParams {
            epsilon1: 1.0,
            epsilon2: 1.0,
            a: 1.0,
            b: 1.0,
            c: 1.0,
            translation: Vector3::zeros(),
            rotation: Rotation3::identity(),
            template_index: 0,
            fit_error: 0.0,
        };

        assert!(sq.is_inside(Vector3::new(0.0, 0.0, 0.0)));
        assert!(!sq.is_inside(Vector3::new(2.0, 0.0, 0.0)));
    }

    #[test]
    fn test_jacobian_consistency_with_finite_difference() {
        // Verify analytical Jacobian against finite differences
        let e1 = 1.0;
        let e2 = 1.0;
        let rotation = Rotation3::identity();
        let params = pack_params(1.0, 1.0, 1.0, Vector3::zeros());
        let pt = Vector3::new(0.5, 0.3, 0.7);

        let analytic = compute_jacobian_row(pt, &params, e1, e2, &rotation);

        // Finite difference
        let eps = 1e-4_f32;
        let mut fd = Vector6::zeros();
        for i in 0..6 {
            let mut p_plus = params;
            let mut p_minus = params;
            p_plus[i] += eps;
            p_minus[i] -= eps;

            let (a_p, b_p, c_p, t_p) = unpack_params(&p_plus);
            let (a_m, b_m, c_m, t_m) = unpack_params(&p_minus);

            let local_p = rotation * (pt - t_p);
            let local_m = rotation * (pt - t_m);

            let f_plus = evaluate_f(local_p.x, local_p.y, local_p.z, a_p, b_p, c_p, e1, e2);
            let f_minus = evaluate_f(local_m.x, local_m.y, local_m.z, a_m, b_m, c_m, e1, e2);

            fd[i] = (f_plus - f_minus) / (2.0 * eps);
        }

        for i in 0..6 {
            let rel_err = if analytic[i].abs() > 1e-6 {
                (analytic[i] - fd[i]).abs() / analytic[i].abs()
            } else {
                (analytic[i] - fd[i]).abs()
            };
            assert!(
                rel_err < 0.1,
                "Jacobian[{}] mismatch: analytic={:.6}, fd={:.6}, rel_err={:.4}",
                i, analytic[i], fd[i], rel_err
            );
        }
    }

    #[test]
    fn test_jacobian_consistency_box_template() {
        // Same as test_jacobian_consistency_with_finite_difference but with box template (e1=0.1, e2=0.1)
        let e1 = 0.1;
        let e2 = 0.1;
        let rotation = Rotation3::identity();
        let params = pack_params(1.0, 1.0, 1.0, Vector3::zeros());
        let pt = Vector3::new(0.5, 0.3, 0.7);

        let analytic = compute_jacobian_row(pt, &params, e1, e2, &rotation);

        // Finite difference
        let eps = 1e-4_f32;
        let mut fd = Vector6::zeros();
        for i in 0..6 {
            let mut p_plus = params;
            let mut p_minus = params;
            p_plus[i] += eps;
            p_minus[i] -= eps;

            let (a_p, b_p, c_p, t_p) = unpack_params(&p_plus);
            let (a_m, b_m, c_m, t_m) = unpack_params(&p_minus);

            let local_p = rotation * (pt - t_p);
            let local_m = rotation * (pt - t_m);

            let f_plus = evaluate_f(local_p.x, local_p.y, local_p.z, a_p, b_p, c_p, e1, e2);
            let f_minus = evaluate_f(local_m.x, local_m.y, local_m.z, a_m, b_m, c_m, e1, e2);

            fd[i] = (f_plus - f_minus) / (2.0 * eps);
        }

        for i in 0..6 {
            let rel_err = if analytic[i].abs() > 1e-6 {
                (analytic[i] - fd[i]).abs() / analytic[i].abs()
            } else {
                (analytic[i] - fd[i]).abs()
            };
            assert!(
                rel_err < 0.1,
                "Box Jacobian[{}] mismatch: analytic={:.6}, fd={:.6}, rel_err={:.4}",
                i, analytic[i], fd[i], rel_err
            );
        }
    }

    #[test]
    fn test_jacobian_consistency_cylinder_template() {
        // Same as test_jacobian_consistency_with_finite_difference but with cylinder template (e1=0.1, e2=1.0)
        let e1 = 0.1;
        let e2 = 1.0;
        let rotation = Rotation3::identity();
        let params = pack_params(1.0, 1.0, 1.0, Vector3::zeros());
        let pt = Vector3::new(0.5, 0.3, 0.7);

        let analytic = compute_jacobian_row(pt, &params, e1, e2, &rotation);

        // Finite difference
        let eps = 1e-4_f32;
        let mut fd = Vector6::zeros();
        for i in 0..6 {
            let mut p_plus = params;
            let mut p_minus = params;
            p_plus[i] += eps;
            p_minus[i] -= eps;

            let (a_p, b_p, c_p, t_p) = unpack_params(&p_plus);
            let (a_m, b_m, c_m, t_m) = unpack_params(&p_minus);

            let local_p = rotation * (pt - t_p);
            let local_m = rotation * (pt - t_m);

            let f_plus = evaluate_f(local_p.x, local_p.y, local_p.z, a_p, b_p, c_p, e1, e2);
            let f_minus = evaluate_f(local_m.x, local_m.y, local_m.z, a_m, b_m, c_m, e1, e2);

            fd[i] = (f_plus - f_minus) / (2.0 * eps);
        }

        for i in 0..6 {
            let abs_diff = (analytic[i] - fd[i]).abs();
            let rel_err = if analytic[i].abs() > 1e-6 {
                abs_diff / analytic[i].abs()
            } else {
                abs_diff
            };
            assert!(
                rel_err < 0.1 || abs_diff < 1e-3,
                "Cylinder Jacobian[{}] mismatch: analytic={:.6}, fd={:.6}, rel_err={:.4}",
                i, analytic[i], fd[i], rel_err
            );
        }
    }

    #[test]
    fn test_taubin_distance_accuracy() {
        // Unit sphere: a=b=c=1, e1=e2=1
        let sq = SuperquadricParams {
            epsilon1: 1.0,
            epsilon2: 1.0,
            a: 1.0,
            b: 1.0,
            c: 1.0,
            translation: Vector3::zeros(),
            rotation: Rotation3::identity(),
            template_index: 0,
            fit_error: 0.0,
        };

        // At (2,0,0): true distance from surface = 1.0, should be within 15%
        let d1 = sq.taubin_distance(Vector3::new(2.0, 0.0, 0.0));
        let err1 = (d1 - 1.0).abs() / 1.0;
        assert!(
            err1 < 0.15,
            "Taubin at (2,0,0): got {:.4}, expected ~1.0, relative error {:.4}",
            d1, err1
        );

        // At (1.5,0,0): true distance from surface = 0.5, should be within 10%
        let d2 = sq.taubin_distance(Vector3::new(1.5, 0.0, 0.0));
        let err2 = (d2 - 0.5).abs() / 0.5;
        assert!(
            err2 < 0.10,
            "Taubin at (1.5,0,0): got {:.4}, expected ~0.5, relative error {:.4}",
            d2, err2
        );

        // At (0,0,0): true distance from surface = -1.0, should be within 15%
        let d3 = sq.taubin_distance(Vector3::new(0.0, 0.0, 0.0));
        let err3 = (d3 - (-1.0)).abs() / 1.0;
        assert!(
            err3 < 0.15,
            "Taubin at (0,0,0): got {:.4}, expected ~-1.0, relative error {:.4}",
            d3, err3
        );
    }
}
