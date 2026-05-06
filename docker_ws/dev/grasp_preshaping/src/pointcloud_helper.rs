use crate::config;
use crate::superquadric::SuperquadricParams;
use nalgebra::Vector3;
use rayon::prelude::*;
use std::collections::VecDeque;

#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    pub min: Vector3<f32>,
    pub max: Vector3<f32>,
}

impl Aabb {
    pub fn contains(&self, p: &Vector3<f32>) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    pub fn from_points(points: &[Vector3<f32>]) -> Self {
        assert!(
            !points.is_empty(),
            "cannot compute AABB from empty point set"
        );
        let mut min = points[0];
        let mut max = points[0];
        for p in &points[1..] {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }
        Self { min, max }
    }

    pub fn inflate(&mut self, radius: f32) {
        self.min.x -= radius;
        self.min.y -= radius;
        self.min.z -= radius;
        self.max.x += radius;
        self.max.y += radius;
        self.max.z += radius;
    }

    pub fn enforce_min_dims(&mut self, min_dims: &Vector3<f32>) {
        let center = (self.min + self.max) * 0.5;
        let half = (self.max - self.min) * 0.5;

        let new_half = Vector3::new(
            half.x.max(min_dims.x * 0.5),
            half.y.max(min_dims.y * 0.5),
            half.z.max(min_dims.z * 0.5),
        );

        self.min = center - new_half;
        self.max = center + new_half;
    }

    pub fn clip_max_dims(&mut self, max_dims: &Vector3<f32>, anchor: &Vector3<f32>) {
        for (axis, max_dim) in [(0, max_dims.x), (1, max_dims.y), (2, max_dims.z)] {
            let size = self.max[axis] - self.min[axis];
            if size <= max_dim {
                continue;
            }
            let a = anchor[axis];
            let lo = self.min[axis];
            let hi = self.max[axis];
            if a >= lo && a <= lo + max_dim {
                self.max[axis] = lo + max_dim;
            } else if a >= hi - max_dim && a <= hi {
                self.min[axis] = hi - max_dim;
            } else {
                self.min[axis] = a - max_dim * 0.5;
                self.max[axis] = a + max_dim * 0.5;
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct Camera {
    pub position: Vector3<f32>,
}

#[derive(Debug, Clone)]
pub struct PointCloud {
    pub points: Vec<Vector3<f32>>,
}

impl PointCloud {
    pub fn new(points: Vec<Vector3<f32>>) -> Self {
        Self { points }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

#[derive(Debug, Clone)]
pub struct MortonPoint {
    pub morton_id: u64,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub struct Tsdf {
    data: Vec<f32>,
    width: usize,
    height: usize,
    depth: usize,
    resolution_m: f32,
    origin: Vector3<f32>,
}

impl Tsdf {
    pub fn get_distance(&self, x: f32, y: f32, z: f32) -> f32 {
        let gx = (x - self.origin.x) / self.resolution_m;
        let gy = (y - self.origin.y) / self.resolution_m;
        let gz = (z - self.origin.z) / self.resolution_m;

        if gx < 0.0 || gx >= (self.width - 1) as f32
            || gy < 0.0 || gy >= (self.height - 1) as f32
            || gz < 0.0 || gz >= (self.depth - 1) as f32
        {
            return f32::MAX;
        }

        let x0 = (gx.floor() as usize).min(self.width - 1);
        let y0 = (gy.floor() as usize).min(self.height - 1);
        let z0 = (gz.floor() as usize).min(self.depth - 1);
        let x1 = (x0 + 1).min(self.width - 1);
        let y1 = (y0 + 1).min(self.height - 1);
        let z1 = (z0 + 1).min(self.depth - 1);

        let tx = (gx - x0 as f32).clamp(0.0, 1.0);
        let ty = (gy - y0 as f32).clamp(0.0, 1.0);
        let tz = (gz - z0 as f32).clamp(0.0, 1.0);

        let stride_y = self.width;
        let stride_z = self.width * self.height;

        let c000 = self.data[x0 + y0 * stride_y + z0 * stride_z];
        let c100 = self.data[x1 + y0 * stride_y + z0 * stride_z];
        let c010 = self.data[x0 + y1 * stride_y + z0 * stride_z];
        let c110 = self.data[x1 + y1 * stride_y + z0 * stride_z];
        let c001 = self.data[x0 + y0 * stride_y + z1 * stride_z];
        let c101 = self.data[x1 + y0 * stride_y + z1 * stride_z];
        let c011 = self.data[x0 + y1 * stride_y + z1 * stride_z];
        let c111 = self.data[x1 + y1 * stride_y + z1 * stride_z];

        if c000 == f32::MAX
            || c100 == f32::MAX
            || c010 == f32::MAX
            || c110 == f32::MAX
            || c001 == f32::MAX
            || c101 == f32::MAX
            || c011 == f32::MAX
            || c111 == f32::MAX
        {
            return f32::MAX;
        }

        let lerp = |a: f32, b: f32, t: f32| a + (b - a) * t;

        let c00 = lerp(c000, c100, tx);
        let c10 = lerp(c010, c110, tx);
        let c01 = lerp(c001, c101, tx);
        let c11 = lerp(c011, c111, tx);

        let c0 = lerp(c00, c10, ty);
        let c1 = lerp(c01, c11, ty);

        lerp(c0, c1, tz)
    }

    pub fn get_surface_normal(&self, x: f32, y: f32, z: f32) -> Vector3<f32> {
        let h = self.resolution_m * 0.5;

        let d_xp = self.get_distance(x + h, y, z);
        let d_xn = self.get_distance(x - h, y, z);
        let d_yp = self.get_distance(x, y + h, z);
        let d_yn = self.get_distance(x, y - h, z);
        let d_zp = self.get_distance(x, y, z + h);
        let d_zn = self.get_distance(x, y, z - h);

        if d_xp == f32::MAX
            || d_xn == f32::MAX
            || d_yp == f32::MAX
            || d_yn == f32::MAX
            || d_zp == f32::MAX
            || d_zn == f32::MAX
        {
            return Vector3::new(0.0, 0.0, 1.0);
        }

        let grad = Vector3::new(d_xp - d_xn, d_yp - d_yn, d_zp - d_zn);
        let norm = grad.norm();
        if norm < 1e-10 {
            Vector3::new(0.0, 0.0, 1.0)
        } else {
            grad / norm
        }
    }

    /// Returns the raw signed-distance data as a flat slice.
    /// Layout: x-major, then y, then z (row-major with strides width, width*height).
    pub fn data(&self) -> &[f32] {
        &self.data
    }

    /// Returns the (width, height, depth) grid dimensions.
    pub fn dimensions(&self) -> (usize, usize, usize) {
        (self.width, self.height, self.depth)
    }

    /// Returns the grid resolution in metres.
    pub fn resolution(&self) -> f32 {
        self.resolution_m
    }

    /// Returns the world-space origin (corner of voxel [0,0,0]).
    pub fn origin(&self) -> Vector3<f32> {
        self.origin
    }
}

pub fn prune(pc: &PointCloud, aabb: Option<Aabb>) -> PointCloud {
    match aabb {
        Some(aabb) => {
            let points: Vec<Vector3<f32>> = pc
                .points
                .par_iter()
                .filter(|p| aabb.contains(p))
                .copied()
                .collect();
            PointCloud::new(points)
        }
        None => pc.clone(),
    }
}

fn split_by_3(x: u16) -> u64 {
    let mut result: u64 = 0;
    for i in 0..16 {
        result |= (((x >> i) & 1) as u64) << (3 * i);
    }
    result
}

fn decode_morton(morton: u64) -> (u16, u16, u16) {
    let mut gx = 0u16;
    let mut gy = 0u16;
    let mut gz = 0u16;
    for i in 0..16 {
        gx |= (((morton >> (3 * i)) & 1) as u16) << i;
        gy |= (((morton >> (3 * i + 1)) & 1) as u16) << i;
        gz |= (((morton >> (3 * i + 2)) & 1) as u16) << i;
    }
    (gx, gy, gz)
}

// Morton codes interleave x/y/z bits so that spatially-close points have close codes.
// Sorting by morton code gives a Z-order curve, yielding cache-friendly TSDF traversal.
pub fn morton(pc: &PointCloud, resolution_m: f32) -> (Vec<MortonPoint>, Vec<usize>, Vector3<f32>) {
    assert!(!pc.is_empty(), "empty point cloud");
    assert!(resolution_m > 0.0, "resolution must be positive");

    let mut min = pc.points[0];
    let mut max = pc.points[0];
    for p in &pc.points {
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        min.z = min.z.min(p.z);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
        max.z = max.z.max(p.z);
    }

    let mut morton_points: Vec<MortonPoint> = pc
        .points
        .par_iter()
        .map(|p| {
            let gx = ((p.x - min.x) / resolution_m)
                .floor()
                .max(0.0)
                .min(65535.0) as u16;
            let gy = ((p.y - min.y) / resolution_m)
                .floor()
                .max(0.0)
                .min(65535.0) as u16;
            let gz = ((p.z - min.z) / resolution_m)
                .floor()
                .max(0.0)
                .min(65535.0) as u16;
            let morton_id = split_by_3(gx) | (split_by_3(gy) << 1) | (split_by_3(gz) << 2);
            MortonPoint {
                morton_id,
                x: p.x,
                y: p.y,
                z: p.z,
            }
        })
        .collect();

    morton_points.par_sort_by_key(|mp| mp.morton_id);

    let mut offsets = Vec::new();
    offsets.push(0);
    for i in 1..morton_points.len() {
        if morton_points[i].morton_id != morton_points[i - 1].morton_id {
            offsets.push(i);
        }
    }
    offsets.push(morton_points.len());

    (morton_points, offsets, min)
}

pub fn get_tsdf(
    morton_array: &[MortonPoint],
    offsets: &[usize],
    truncation_cells: usize,
    start_coords: Vector3<f32>,
    resolution_m: f32,
    cameras: &[Camera],
    sq_params: Option<&SuperquadricParams>,
) -> Tsdf {
    assert!(!morton_array.is_empty(), "empty morton array");
    assert!(resolution_m > 0.0, "resolution must be positive");

    let mut max_gx: usize = 0;
    let mut max_gy: usize = 0;
    let mut max_gz: usize = 0;
    for group in 0..offsets.len() - 1 {
        let (gx, gy, gz) = decode_morton(morton_array[offsets[group]].morton_id);
        max_gx = max_gx.max(gx as usize);
        max_gy = max_gy.max(gy as usize);
        max_gz = max_gz.max(gz as usize);
    }

    let trunc = truncation_cells;
    let width = max_gx + 1 + 2 * trunc;
    let height = max_gy + 1 + 2 * trunc;
    let depth = max_gz + 1 + 2 * trunc;
    let total = width * height * depth;

    let origin =
        start_coords - Vector3::new(trunc as f32, trunc as f32, trunc as f32) * resolution_m;

    let mut distance = vec![f32::MAX; total];
    let mut nearest = vec![0u32; total];

    let stride_y = width;
    let stride_z = width * height;

    let mut queue = VecDeque::with_capacity(total / 4);
    for group in 0..offsets.len() - 1 {
        let start = offsets[group];
        let (gx, gy, gz) = decode_morton(morton_array[start].morton_id);
        let px = gx as usize + trunc;
        let py = gy as usize + trunc;
        let pz = gz as usize + trunc;
        let idx = px + py * stride_y + pz * stride_z;
        distance[idx] = 0.0;
        nearest[idx] = start as u32;
        queue.push_back((px, py, pz));
    }

    let neighbor_offsets: [(isize, isize, isize); 6] = [
        (1, 0, 0),
        (-1, 0, 0),
        (0, 1, 0),
        (0, -1, 0),
        (0, 0, 1),
        (0, 0, -1),
    ];

    while let Some((cx, cy, cz)) = queue.pop_front() {
        let cidx = cx + cy * stride_y + cz * stride_z;
        let cdist = distance[cidx];
        if cdist >= truncation_cells as f32 {
            continue;
        }

        for &(dx, dy, dz) in &neighbor_offsets {
            let nx = cx as isize + dx;
            let ny = cy as isize + dy;
            let nz = cz as isize + dz;
            if nx < 0 || ny < 0 || nz < 0 {
                continue;
            }
            let (nx, ny, nz) = (nx as usize, ny as usize, nz as usize);
            if nx >= width || ny >= height || nz >= depth {
                continue;
            }
            let nidx = nx + ny * stride_y + nz * stride_z;
            let new_dist = cdist + 1.0;
            if new_dist < distance[nidx] {
                distance[nidx] = new_dist;
                nearest[nidx] = nearest[cidx];
                queue.push_back((nx, ny, nz));
            }
        }
    }

    let n_cams = cameras.len();
    if n_cams > 0 {
        // Determine TSDF sign using per-camera ray-based occlusion checking.
        // For each voxel, for each camera, we find the surface point closest
        // to the camera-to-voxel ray (by perpendicular distance). If the voxel
        // is farther along the ray than this surface point, the surface occludes
        // the voxel from that camera → "inside" vote. If closer → "outside" vote.
        //
        // A voxel is negative (inside the object) when at least one camera
        // confirms occlusion AND no camera sees it as unoccluded. This correctly
        // handles opposing cameras where the globally-nearest surface point gives
        // misleading information for one camera.

        distance
            .par_iter_mut()
            .enumerate()
            .for_each(|(flat_idx, dist)| {
                if *dist == f32::MAX || *dist == 0.0 {
                    return;
                }

                let gz = flat_idx / stride_z;
                let rem = flat_idx - gz * stride_z;
                let gy = rem / stride_y;
                let gx = rem % stride_y;

                let vw = origin + Vector3::new(gx as f32, gy as f32, gz as f32) * resolution_m;

                let nearest_idx = nearest[flat_idx] as usize;
                let mp = &morton_array[nearest_idx];
                let sp = Vector3::new(mp.x, mp.y, mp.z);

                let mut inside_votes = 0usize;
                let mut outside_votes = 0usize;

                for cam in cameras {
                    let to_voxel_from_cam = vw - cam.position;
                    let ray_len = to_voxel_from_cam.norm();
                    if ray_len < 1e-10 {
                        outside_votes += 1;
                        continue;
                    }
                    let ray_dir = to_voxel_from_cam / ray_len;

                    let to_surf = sp - cam.position;
                    let proj = to_surf.dot(&ray_dir);
                    
                    if proj < 0.0 {
                        // Surface point is behind the camera.
                        // Can't reliably use it, assume outside.
                        outside_votes += 1;
                        continue; 
                    }

                    // Alignment check: the direction from the surface point
                    // to the voxel should be consistent with the camera-to-voxel
                    // ray direction.
                    let to_voxel_from_surf = vw - sp;
                    let to_voxel_from_surf_len = to_voxel_from_surf.norm();
                    if to_voxel_from_surf_len > 1e-10 {
                        let surf_dir = to_voxel_from_surf / to_voxel_from_surf_len;
                        let alignment = ray_dir.dot(&surf_dir);
                        if alignment < config::RAY_ALIGNMENT_THRESHOLD {
                            // Misaligned, means we are wrapping around the object
                            // and the nearest point is on a surface facing away
                            // from this camera ray path. We can't trust it for this camera.
                            // In a multi-camera setup, another camera might see it better.
                            continue;
                        }
                    }

                    let perp_sq = to_surf.norm_squared() - proj * proj;
                    let max_perp = config::TRUNCATION_CELLS as f32 * resolution_m;
                    
                    // Only vote if the nearest point is close enough to the ray
                    if (perp_sq.sqrt()) < max_perp {
                        let voxel_proj = ray_len;
                        if voxel_proj > proj {
                            inside_votes += 1;
                        } else {
                            outside_votes += 1;
                        }
                    }
                }

                // A voxel is inside when at least one camera confirms it is behind
                // the surface and no camera sees it in front.
                //
                // When cameras disagree (both inside and outside votes), use
                // the superquadric as a tiebreaker if available.
                if inside_votes > 0 && outside_votes == 0 {
                    *dist = -*dist;
                } else if inside_votes > 0 && outside_votes > 0 {
                    // Opposing cameras disagree — use superquadric tiebreaker.
                    if let Some(sq) = sq_params {
                        if sq.is_inside(vw) {
                            *dist = -*dist;
                        }
                    }
                    // If no superquadric available, leave sign positive (outside).
                }
            });
    }

    // ── Unified SQ sign-correction and distance blend ────────────────────
    //
    // After Domain A (camera authority with ray-based sign), apply SQ
    // corrections. The key insight: the SQ sign is authoritative for
    // determining inside/outside, while the camera distance is authoritative
    // near the visible surface.
    //
    // For each voxel:
    // - If camera and SQ signs agree: keep camera data (possibly blend
    //   distances in the outer band for smoothness)
    // - If camera and SQ signs disagree: the camera sign is wrong (backside
    //   voxel marked as outside). Use SQ sign and blend distances based on
    //   proximity to the visible surface.
    //
    // Performance optimization: only evaluate the expensive taubin_distance
    // on voxels that could possibly need SQ correction. These are:
    //   - Voxels with negative distance (inside the object)
    //   - Voxels with f32::MAX (unobserved)
    //   - Voxels within truncation_cells of either of the above
    // Voxels that are positive and far from any negative/unobserved region
    // will have signs that agree with SQ (both "outside"), so the SQ pass
    // would be a no-op for them.
    if let Some(sq) = sq_params {
        let blend_start_agree = (truncation_cells - config::SQ_BLEND_DELTA_CELLS) as f32;
        let blend_end_agree = truncation_cells as f32;
        let blend_start_disagree = config::SQ_MIN_SIGN_OVERRIDE_CELLS as f32;
        let blend_end_disagree = (truncation_cells - 1) as f32; // Full SQ at trunc-1 cells

        // Build a sparse mask of voxels that need SQ evaluation.
        // Phase 1: mark negative and unobserved voxels.
        let mut sq_mask = vec![false; total];
        for (i, &d) in distance.iter().enumerate() {
            if d < 0.0 || d == f32::MAX {
                sq_mask[i] = true;
            }
        }

        // Phase 2: dilate the mask by truncation_cells in all 3D directions.
        // This captures the transition zone where blending occurs.
        // We do this in-place by scanning the mask and marking neighbors.
        // Multiple dilation passes of 1 cell each are simpler than variable-radius.
        for _pass in 0..truncation_cells {
            let mut next_mask = sq_mask.clone();
            for gz in 0..depth {
                for gy in 0..height {
                    for gx in 0..width {
                        let idx = gx + gy * stride_y + gz * stride_z;
                        if sq_mask[idx] {
                            // Mark 6-connected neighbors
                            if gx > 0 { next_mask[idx - 1] = true; }
                            if gx + 1 < width { next_mask[idx + 1] = true; }
                            if gy > 0 { next_mask[idx - stride_y] = true; }
                            if gy + 1 < height { next_mask[idx + stride_y] = true; }
                            if gz > 0 { next_mask[idx - stride_z] = true; }
                            if gz + 1 < depth { next_mask[idx + stride_z] = true; }
                        }
                    }
                }
            }
            sq_mask = next_mask;
        }

        distance
            .par_iter_mut()
            .enumerate()
            .for_each(|(flat_idx, dist)| {
                // Skip voxels outside the SQ evaluation mask
                if !sq_mask[flat_idx] {
                    return;
                }

                let cam_tdf = *dist;

                // Skip surface voxels and truly unvisited voxels
                if cam_tdf == 0.0 || cam_tdf == f32::MAX {
                    if cam_tdf == f32::MAX {
                        // Shouldn't happen given grid sizing, but handle it
                        let gz = flat_idx / stride_z;
                        let rem = flat_idx - gz * stride_z;
                        let gy = rem / stride_y;
                        let gx = rem % stride_y;
                        let vw = origin
                            + Vector3::new(gx as f32, gy as f32, gz as f32)
                                * resolution_m;
                        let sq_dist = sq.taubin_distance(vw);
                        let sq_tdf = sq_dist / resolution_m;
                        *dist = sq_tdf.clamp(
                            -(truncation_cells as f32),
                            truncation_cells as f32,
                        );
                    }
                    return;
                }

                // Compute world position of this voxel
                let gz = flat_idx / stride_z;
                let rem = flat_idx - gz * stride_z;
                let gy = rem / stride_y;
                let gx = rem % stride_y;
                let vw =
                    origin + Vector3::new(gx as f32, gy as f32, gz as f32) * resolution_m;

                // SQ distance and sign
                let sq_dist = sq.taubin_distance(vw);
                let sq_tdf = sq_dist / resolution_m;
                let sq_tdf_clamped =
                    sq_tdf.clamp(-(truncation_cells as f32), truncation_cells as f32);
                let sq_inside = sq.evaluate(vw) < 0.0;

                let abs_cam = cam_tdf.abs();
                let cam_inside = cam_tdf < 0.0;
                let signs_agree = cam_inside == sq_inside;

                if !signs_agree {
                    // Camera and SQ disagree on sign.
                    // Trust SQ for sign, blend distances based on proximity
                    // to the visible surface.
                    if abs_cam <= blend_start_disagree {
                        // Very close to visible surface — camera distance is
                        // good, but flip the sign to match SQ.
                        *dist = if sq_inside { -abs_cam } else { abs_cam };
                    } else if abs_cam >= blend_end_disagree {
                        // Far from surface — SQ is fully authoritative.
                        *dist = sq_tdf_clamped;
                    } else {
                        // Transition zone — blend absolute distances, use SQ sign.
                        let t = (abs_cam - blend_start_disagree)
                            / (blend_end_disagree - blend_start_disagree);
                        let t = t.clamp(0.0, 1.0);
                        // Smoothstep: w goes from 1 (camera) to 0 (SQ)
                        let w = 1.0 - t * t * (3.0 - 2.0 * t);
                        let blended_abs = w * abs_cam + (1.0 - w) * sq_tdf_clamped.abs();
                        *dist = if sq_inside { -blended_abs } else { blended_abs };
                    }
                } else {
                    // Signs agree — blend distances in the outer band only.
                    if abs_cam >= blend_start_agree && abs_cam <= blend_end_agree {
                        let t = (abs_cam - blend_start_agree)
                            / (blend_end_agree - blend_start_agree);
                        let t = t.clamp(0.0, 1.0);
                        let w = 1.0 - t * t * (3.0 - 2.0 * t);
                        *dist = w * cam_tdf + (1.0 - w) * sq_tdf_clamped;
                    }
                }
            });
    }

    Tsdf {
        data: distance,
        width,
        height,
        depth,
        resolution_m,
        origin,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prune_with_aabb_filters_points() {
        let pc = PointCloud::new(vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(5.0, 5.0, 5.0),
            Vector3::new(10.0, 10.0, 10.0),
        ]);
        let aabb = Aabb {
            min: Vector3::new(-1.0, -1.0, -1.0),
            max: Vector3::new(6.0, 6.0, 6.0),
        };
        let pruned = prune(&pc, Some(aabb));
        assert_eq!(pruned.len(), 2);
    }

    #[test]
    fn morton_code_roundtrip() {
        for gx in [0u16, 1, 100, 255, 1000, 65535] {
            for gy in [0u16, 1, 42, 999] {
                for gz in [0u16, 1, 7, 5432] {
                    let code = split_by_3(gx) | (split_by_3(gy) << 1) | (split_by_3(gz) << 2);
                    let (rx, ry, rz) = decode_morton(code);
                    assert_eq!(rx, gx, "gx mismatch for ({}, {}, {})", gx, gy, gz);
                    assert_eq!(ry, gy, "gy mismatch for ({}, {}, {})", gx, gy, gz);
                    assert_eq!(rz, gz, "gz mismatch for ({}, {}, {})", gx, gy, gz);
                }
            }
        }
    }

    #[test]
    fn morton_sorts_by_code_and_builds_offsets() {
        let pc = PointCloud::new(vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.5, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
        ]);
        let (sorted, offsets, _start) = morton(&pc, 1.0);
        assert_eq!(sorted.len(), 3);
        assert!(offsets.len() >= 2);
        for i in 1..sorted.len() {
            assert!(sorted[i].morton_id >= sorted[i - 1].morton_id);
        }
        assert_eq!(*offsets.last().unwrap(), sorted.len());
    }

    #[test]
    fn tsdf_surface_voxels_have_zero_distance() {
        let pc = PointCloud::new(vec![Vector3::new(5.0, 5.0, 5.0)]);
        let (morton_arr, offsets, start) = morton(&pc, 1.0);
        let tsdf = get_tsdf(&morton_arr, &offsets, 3, start, 1.0, &[], None);

        let d = tsdf.get_distance(5.0, 5.0, 5.0);
        assert!(d.abs() < 0.01, "surface distance should be ~0, got {}", d);
    }

    #[test]
    fn tsdf_distance_increases_away_from_surface() {
        let pc = PointCloud::new(vec![Vector3::new(10.0, 10.0, 10.0)]);
        let (morton_arr, offsets, start) = morton(&pc, 1.0);
        let tsdf = get_tsdf(&morton_arr, &offsets, 5, start, 1.0, &[], None);

        let d0 = tsdf.get_distance(10.0, 10.0, 10.0);
        let d1 = tsdf.get_distance(11.0, 10.0, 10.0);
        let d2 = tsdf.get_distance(12.0, 10.0, 10.0);
        assert!(d0.abs() < 0.01, "d0 = {}", d0);
        assert!(d1 > 0.5, "d1 = {}", d1);
        assert!(d2 > d1, "d2 = {}, d1 = {}", d2, d1);
    }

    #[test]
    fn tsdf_truncation_bounds_distance() {
        let pc = PointCloud::new(vec![Vector3::new(20.0, 20.0, 20.0)]);
        let trunc = 3;
        let (morton_arr, offsets, start) = morton(&pc, 1.0);
        let tsdf = get_tsdf(&morton_arr, &offsets, trunc, start, 1.0, &[], None);

        let d_far = tsdf.get_distance(30.0, 20.0, 20.0);
        assert_eq!(d_far, f32::MAX, "beyond truncation should be f32::MAX");
    }

    #[test]
    fn tsdf_sign_negative_inside_with_cameras() {
        let pc = PointCloud::new(vec![
            Vector3::new(10.0, 10.0, 10.0),
            Vector3::new(12.0, 10.0, 10.0),
            Vector3::new(10.0, 12.0, 10.0),
            Vector3::new(12.0, 12.0, 10.0),
            Vector3::new(10.0, 10.0, 12.0),
            Vector3::new(12.0, 10.0, 12.0),
            Vector3::new(10.0, 12.0, 12.0),
            Vector3::new(12.0, 12.0, 12.0),
            Vector3::new(11.0, 11.0, 10.0),
        ]);
        let cameras = vec![Camera {
            position: Vector3::new(11.0, 11.0, 8.0),
        }];
        let (morton_arr, offsets, start) = morton(&pc, 1.0);
        let tsdf = get_tsdf(&morton_arr, &offsets, 5, start, 1.0, &cameras, None);

        let d_inside = tsdf.get_distance(11.0, 11.0, 11.0);
        assert!(
            d_inside < 0.0,
            "point inside cube shell should be negative, got {}",
            d_inside
        );
    }

    #[test]
    fn tsdf_sign_negative_inside_with_opposite_cameras() {
        // Cube shell centered at (11,11,11) with cameras on opposite sides along Z.
        // This tests the fix for the sign voting threshold: with 2 opposing cameras,
        // a voxel at the center is "behind" the surface from only one camera, so a
        // majority vote (behind_count > n_cams/2) fails. The fix uses inside_votes > 0.
        let pc = PointCloud::new(vec![
            Vector3::new(10.0, 10.0, 10.0),
            Vector3::new(12.0, 10.0, 10.0),
            Vector3::new(10.0, 12.0, 10.0),
            Vector3::new(12.0, 12.0, 10.0),
            Vector3::new(10.0, 10.0, 12.0),
            Vector3::new(12.0, 10.0, 12.0),
            Vector3::new(10.0, 12.0, 12.0),
            Vector3::new(12.0, 12.0, 12.0),
            Vector3::new(11.0, 11.0, 10.0),
            Vector3::new(11.0, 11.0, 12.0),
        ]);
        let cameras = vec![
            Camera {
                position: Vector3::new(11.0, 11.0, 8.0),
            },
            Camera {
                position: Vector3::new(11.0, 11.0, 14.0),
            },
        ];
        let (morton_arr, offsets, start) = morton(&pc, 1.0);
        let tsdf = get_tsdf(&morton_arr, &offsets, 5, start, 1.0, &cameras, None);

        let d_inside = tsdf.get_distance(11.0, 11.0, 11.0);
        assert!(
            d_inside < 0.0,
            "point inside cube shell should be negative with opposite cameras, got {}",
            d_inside
        );

        // Voxels outside the shell should remain positive.
        let d_outside = tsdf.get_distance(11.0, 11.0, 8.5);
        assert!(
            d_outside > 0.0,
            "point outside cube shell should be positive, got {}",
            d_outside
        );
    }

    #[test]
    fn surface_normal_points_outward() {
        let pc = PointCloud::new(vec![Vector3::new(10.0, 10.0, 10.0)]);
        let (morton_arr, offsets, start) = morton(&pc, 1.0);
        let tsdf = get_tsdf(&morton_arr, &offsets, 5, start, 1.0, &[], None);

        let normal = tsdf.get_surface_normal(12.0, 10.0, 10.0);
        let dir = Vector3::new(1.0, 0.0, 0.0);
        let dot = normal.dot(&dir);
        assert!(
            dot > 0.5,
            "normal should point roughly in +x away from surface, dot = {}",
            dot
        );
    }

    #[test]
    fn aabb_enforce_min_dims_centers_when_too_small() {
        let aabb = Aabb {
            min: Vector3::new(0.0, 0.0, 0.0),
            max: Vector3::new(0.01, 0.01, 0.01),
        };
        let mut aabb = aabb;
        aabb.enforce_min_dims(&Vector3::new(0.1, 0.1, 0.1));
        let size = aabb.max - aabb.min;
        assert!((size.x - 0.1).abs() < 1e-6);
        assert!((size.y - 0.1).abs() < 1e-6);
        assert!((size.z - 0.1).abs() < 1e-6);
        let center = (aabb.min + aabb.max) * 0.5;
        assert!((center.x - 0.005).abs() < 1e-6);
    }

    #[test]
    fn prune_and_build_tsdf_in_roi() {
        let pc = PointCloud::new(vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 10.0, 10.0),
            Vector3::new(100.0, 100.0, 100.0),
        ]);
        let roi = Aabb {
            min: Vector3::new(-1.0, -1.0, -1.0),
            max: Vector3::new(11.0, 11.0, 11.0),
        };
        let pruned = prune(&pc, Some(roi));
        assert_eq!(pruned.len(), 2);
        let (morton_arr, offsets, start) = morton(&pruned, 1.0);
        let tsdf = get_tsdf(&morton_arr, &offsets, 3, start, 1.0, &[], None);
        let d0 = tsdf.get_distance(0.0, 0.0, 0.0);
        let d10 = tsdf.get_distance(10.0, 10.0, 10.0);
        assert!(
            d0.abs() < 0.01,
            "surface point should have ~0 distance, got {}",
            d0
        );
        assert!(
            d10.abs() < 0.01,
            "surface point should have ~0 distance, got {}",
            d10
        );
    }

    #[test]
    fn aabb_clip_max_dims_no_clip_when_within() {
        let mut aabb = Aabb {
            min: Vector3::new(0.0, 0.0, 0.0),
            max: Vector3::new(0.1, 0.1, 0.1),
        };
        let anchor = Vector3::new(0.05, 0.05, 0.05);
        aabb.clip_max_dims(&Vector3::new(0.3, 0.3, 0.3), &anchor);
        assert!((aabb.max.x - 0.1).abs() < 1e-6);
        assert!((aabb.min.x - 0.0).abs() < 1e-6);
    }

    #[test]
    fn aabb_clip_max_dims_clips_anchored_at_min() {
        let mut aabb = Aabb {
            min: Vector3::new(0.0, 0.0, 0.0),
            max: Vector3::new(1.0, 1.0, 1.0),
        };
        let anchor = Vector3::new(0.0, 0.0, 0.0);
        aabb.clip_max_dims(&Vector3::new(0.3, 0.3, 0.3), &anchor);
        assert!((aabb.max.x - 0.3).abs() < 1e-6, "max.x = {}", aabb.max.x);
        assert!((aabb.min.x - 0.0).abs() < 1e-6);
    }

    #[test]
    fn aabb_clip_max_dims_clips_anchored_at_max() {
        let mut aabb = Aabb {
            min: Vector3::new(0.0, 0.0, 0.0),
            max: Vector3::new(1.0, 1.0, 1.0),
        };
        let anchor = Vector3::new(1.0, 1.0, 1.0);
        aabb.clip_max_dims(&Vector3::new(0.3, 0.3, 0.3), &anchor);
        assert!((aabb.min.x - 0.7).abs() < 1e-6, "min.x = {}", aabb.min.x);
        assert!((aabb.max.x - 1.0).abs() < 1e-6);
    }

    #[test]
    fn aabb_clip_max_dims_centers_on_anchor_when_middle() {
        let mut aabb = Aabb {
            min: Vector3::new(-1.0, -1.0, -1.0),
            max: Vector3::new(1.0, 1.0, 1.0),
        };
        let anchor = Vector3::new(0.0, 0.0, 0.0);
        aabb.clip_max_dims(&Vector3::new(0.2, 0.2, 0.2), &anchor);
        assert!((aabb.min.x - (-0.1)).abs() < 1e-6, "min.x = {}", aabb.min.x);
        assert!((aabb.max.x - 0.1).abs() < 1e-6, "max.x = {}", aabb.max.x);
    }

    // ── Superquadric backside integration tests ─────────────────────────

    /// Helper: generate points on the front hemisphere of a sphere.
    fn front_hemisphere_points(center: Vector3<f32>, radius: f32, n: usize) -> PointCloud {
        use std::f32::consts::PI;
        let mut points = Vec::new();
        for i in 0..n {
            let theta = PI * (i as f32 / n as f32); // 0 to PI (front half)
            for j in 0..20 {
                let phi = 2.0 * PI * (j as f32 / 20.0);
                let x = center.x + radius * theta.sin() * phi.cos();
                let y = center.y + radius * theta.sin() * phi.sin();
                let z = center.z + radius * theta.cos();
                points.push(Vector3::new(x, y, z));
            }
        }
        PointCloud::new(points)
    }

    #[test]
    fn tsdf_with_superquadric_fills_backside_voxels() {
        // Front hemisphere observed by a camera in front.
        // With superquadric, voxels deep inside the object (beyond camera
        // visibility) should get correct signs from the superquadric.
        let center = Vector3::new(0.1, 0.1, 0.1);
        let radius = 0.03_f32;
        let pc = front_hemisphere_points(center, radius, 100);
        let cameras = vec![Camera {
            position: Vector3::new(center.x, center.y, center.z - 0.3),
        }];

        let (morton_arr, offsets, start) = morton(&pc, config::TSDF_RESOLUTION_M);

        // With superquadric
        let sq = crate::superquadric::fit_best_superquadric(&pc.points);
        assert!(sq.is_some(), "superquadric fitting should succeed");

        let tsdf_with_sq = get_tsdf(
            &morton_arr, &offsets, config::TRUNCATION_CELLS,
            start, config::TSDF_RESOLUTION_M, &cameras, sq.as_ref(),
        );

        // The center of the sphere should be inside (negative).
        // Without superquadric, this might have wrong sign due to the
        // camera not seeing the backside. With SQ, the sign is correct.
        let d_center = tsdf_with_sq.get_distance(center.x, center.y, center.z);
        assert!(
            d_center < 0.0,
            "center should be negative (inside) with SQ, got {}",
            d_center
        );

        // A point just outside the back of the sphere should be positive.
        // The superquadric provides the distance estimate here.
        let back_outside = center + Vector3::new(0.0, 0.0, radius + 0.01);
        let d_back = tsdf_with_sq.get_distance(back_outside.x, back_outside.y, back_outside.z);
        // This should not be f32::MAX — the SQ fill should have covered it
        // (it's within the truncation band of the surface).
        assert!(
            d_back != f32::MAX,
            "back outside point should have valid distance with SQ, got f32::MAX"
        );
    }

    #[test]
    fn tsdf_superquadric_inside_negative() {
        // With superquadric, a point inside the fitted shape should be negative.
        let center = Vector3::new(0.1, 0.1, 0.1);
        let radius = 0.03_f32;
        let pc = front_hemisphere_points(center, radius, 100);
        let cameras = vec![Camera {
            position: Vector3::new(center.x, center.y, center.z - 0.3),
        }];

        let (morton_arr, offsets, start) = morton(&pc, config::TSDF_RESOLUTION_M);

        let sq = crate::superquadric::fit_best_superquadric(&pc.points);
        assert!(sq.is_some(), "superquadric fitting should succeed");

        let tsdf = get_tsdf(
            &morton_arr, &offsets, config::TRUNCATION_CELLS,
            start, config::TSDF_RESOLUTION_M, &cameras, sq.as_ref(),
        );

        // A point at the center of the sphere should be inside (negative)
        let d_center = tsdf.get_distance(center.x, center.y, center.z);
        assert!(
            d_center < 0.0,
            "center of sphere should be negative (inside), got {}",
            d_center
        );
    }

    #[test]
    fn tsdf_superquadric_sign_tiebreaker_with_opposing_cameras() {
        // When opposing cameras disagree on the sign, the superquadric
        // should act as a tiebreaker to correctly determine inside/outside.
        let center = Vector3::new(0.1, 0.1, 0.1);
        let radius = 0.03_f32;
        let pc = front_hemisphere_points(center, radius, 100);
        let cameras = vec![
            Camera {
                position: Vector3::new(center.x, center.y, center.z - 0.3),
            },
            Camera {
                position: Vector3::new(center.x, center.y, center.z + 0.3),
            },
        ];

        let (morton_arr, offsets, start) = morton(&pc, config::TSDF_RESOLUTION_M);

        let sq = crate::superquadric::fit_best_superquadric(&pc.points);

        let tsdf = get_tsdf(
            &morton_arr, &offsets, config::TRUNCATION_CELLS,
            start, config::TSDF_RESOLUTION_M, &cameras, sq.as_ref(),
        );

        // Center should still be negative (inside) even with opposing cameras
        let d_center = tsdf.get_distance(center.x, center.y, center.z);
        assert!(
            d_center < 0.0,
            "center should be negative with SQ tiebreaker, got {}",
            d_center
        );
    }

    #[test]
    fn tsdf_backside_outside_positive() {
        // Generate a front hemisphere, fit SQ, verify that a point just
        // outside the BACK of the sphere has a POSITIVE TSDF distance.
        let center = Vector3::new(0.1, 0.1, 0.1);
        let radius = 0.03_f32;
        let pc = front_hemisphere_points(center, radius, 100);
        let cameras = vec![Camera {
            position: Vector3::new(center.x, center.y, center.z - 0.3),
        }];

        let (morton_arr, offsets, start) = morton(&pc, config::TSDF_RESOLUTION_M);
        let sq = crate::superquadric::fit_best_superquadric(&pc.points);
        assert!(sq.is_some(), "superquadric fitting should succeed");

        let tsdf = get_tsdf(
            &morton_arr, &offsets, config::TRUNCATION_CELLS,
            start, config::TSDF_RESOLUTION_M, &cameras, sq.as_ref(),
        );

        // A point just outside the back of the sphere should be positive.
        let back_outside = center + Vector3::new(0.0, 0.0, radius + 0.005);
        let d_back = tsdf.get_distance(back_outside.x, back_outside.y, back_outside.z);
        assert!(
            d_back > 0.0,
            "point just outside the BACK of the sphere should have positive TSDF, got {}",
            d_back
        );
    }

    #[test]
    fn tsdf_backside_monotonic() {
        // Verify that TSDF distances on the backside increase monotonically
        // as you move away from the object (no floating markers or sign flips).
        let center = Vector3::new(0.1, 0.1, 0.1);
        let radius = 0.03_f32;
        let pc = front_hemisphere_points(center, radius, 100);
        let cameras = vec![Camera {
            position: Vector3::new(center.x, center.y, center.z - 0.3),
        }];

        let (morton_arr, offsets, start) = morton(&pc, config::TSDF_RESOLUTION_M);
        let sq = crate::superquadric::fit_best_superquadric(&pc.points);
        assert!(sq.is_some(), "superquadric fitting should succeed");

        let tsdf = get_tsdf(
            &morton_arr, &offsets, config::TRUNCATION_CELLS,
            start, config::TSDF_RESOLUTION_M, &cameras, sq.as_ref(),
        );

        // Sample points along the backside (+Z direction from center) and
        // verify monotonic increase in TSDF distance.
        let step = config::TSDF_RESOLUTION_M;
        let mut prev_d: Option<f32> = None;
        for i in 0..(config::TRUNCATION_CELLS + 2) {
            let offset_z = radius + (i as f32) * step;
            let p = center + Vector3::new(0.0, 0.0, offset_z);
            let d = tsdf.get_distance(p.x, p.y, p.z);
            if d == f32::MAX {
                break; // Beyond grid
            }
            if let Some(prev) = prev_d {
                assert!(
                    d >= prev - 0.5, // Allow small numerical tolerance
                    "TSDF should be monotonically non-decreasing on backside: \
                     prev={}, current={} at offset={}",
                    prev, d, offset_z
                );
            }
            prev_d = Some(d);
        }
    }

}
