use nalgebra::Vector3;
use rayon::prelude::*;
use std::collections::VecDeque;

const RAY_ALIGNMENT_THRESHOLD: f32 = 0.8;

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

    pub fn from_xyz_file(path: &str) -> Result<Self, String> {
        let content = std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read '{}': {}", path, e))?;
        let mut points = Vec::new();
        for (line_num, line) in content.lines().enumerate() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() < 3 {
                continue;
            }
            let x: f32 = parts[0]
                .parse()
                .map_err(|e| format!("{}:{}: invalid x: {}", path, line_num + 1, e))?;
            let y: f32 = parts[1]
                .parse()
                .map_err(|e| format!("{}:{}: invalid y: {}", path, line_num + 1, e))?;
            let z: f32 = parts[2]
                .parse()
                .map_err(|e| format!("{}:{}: invalid z: {}", path, line_num + 1, e))?;
            points.push(Vector3::new(x, y, z));
        }
        if points.is_empty() {
            return Err(format!("No valid points found in '{}'", path));
        }
        Ok(Self { points })
    }

    pub fn demo_sphere(center: Vector3<f32>, radius: f32, n_points: usize) -> Self {
        let mut points = Vec::with_capacity(n_points);
        let golden_ratio = (1.0 + 5f32.sqrt()) / 2.0;
        for i in 0..n_points {
            let theta = 2.0 * std::f32::consts::PI * (i as f32 / golden_ratio);
            let phi = (1.0 - 2.0 * (i as f32 + 0.5) / n_points as f32).acos();
            let x = center.x + radius * phi.sin() * theta.cos();
            let y = center.y + radius * phi.sin() * theta.sin();
            let z = center.z + radius * phi.cos();
            points.push(Vector3::new(x, y, z));
        }
        Self { points }
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
    resolution_mm: f32,
    origin: Vector3<f32>,
}

impl Tsdf {
    pub fn get_distance(&self, x: f32, y: f32, z: f32) -> f32 {
        let gx = (x - self.origin.x) / self.resolution_mm;
        let gy = (y - self.origin.y) / self.resolution_mm;
        let gz = (z - self.origin.z) / self.resolution_mm;

        let gx = gx.max(0.0).min((self.width - 1) as f32);
        let gy = gy.max(0.0).min((self.height - 1) as f32);
        let gz = gz.max(0.0).min((self.depth - 1) as f32);

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
        let h = self.resolution_mm * 0.5;

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

pub fn morton(pc: &PointCloud, resolution_mm: f32) -> (Vec<MortonPoint>, Vec<usize>, Vector3<f32>) {
    assert!(!pc.is_empty(), "empty point cloud");
    assert!(resolution_mm > 0.0, "resolution must be positive");

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
            let gx = ((p.x - min.x) / resolution_mm)
                .floor()
                .max(0.0)
                .min(65535.0) as u16;
            let gy = ((p.y - min.y) / resolution_mm)
                .floor()
                .max(0.0)
                .min(65535.0) as u16;
            let gz = ((p.z - min.z) / resolution_mm)
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
    resolution_mm: f32,
    cameras: &[Camera],
) -> Tsdf {
    assert!(!morton_array.is_empty(), "empty morton array");
    assert!(resolution_mm > 0.0, "resolution must be positive");

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
        start_coords - Vector3::new(trunc as f32, trunc as f32, trunc as f32) * resolution_mm;

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

                let vw = origin + Vector3::new(gx as f32, gy as f32, gz as f32) * resolution_mm;

                let mp = &morton_array[nearest[flat_idx] as usize];
                let pw = Vector3::new(mp.x, mp.y, mp.z);

                let mut behind_count = 0usize;
                let mut inside_votes = 0usize;

                for cam in cameras {
                    let d_cv = (vw - cam.position).norm_squared();
                    let d_cp = (pw - cam.position).norm_squared();
                    if d_cv > d_cp {
                        behind_count += 1;
                        let to_voxel = vw - pw;
                        let ray_dir = vw - cam.position;
                        let to_voxel_len = to_voxel.norm();
                        let ray_dir_len = ray_dir.norm();
                        if to_voxel_len > 1e-10 && ray_dir_len > 1e-10 {
                            let alignment = (ray_dir / ray_dir_len).dot(&(to_voxel / to_voxel_len));
                            if alignment > RAY_ALIGNMENT_THRESHOLD {
                                inside_votes += 1;
                            }
                        }
                    }
                }

                if behind_count > n_cams / 2 && inside_votes > behind_count / 2 {
                    *dist = -*dist;
                }
            });
    }

    Tsdf {
        data: distance,
        width,
        height,
        depth,
        resolution_mm,
        origin,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prune_without_aabb_returns_clone() {
        let pc = PointCloud::new(vec![
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
        ]);
        let pruned = prune(&pc, None);
        assert_eq!(pruned.len(), 2);
    }

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
        let tsdf = get_tsdf(&morton_arr, &offsets, 3, start, 1.0, &[]);

        let d = tsdf.get_distance(5.0, 5.0, 5.0);
        assert!(d.abs() < 0.01, "surface distance should be ~0, got {}", d);
    }

    #[test]
    fn tsdf_distance_increases_away_from_surface() {
        let pc = PointCloud::new(vec![Vector3::new(10.0, 10.0, 10.0)]);
        let (morton_arr, offsets, start) = morton(&pc, 1.0);
        let tsdf = get_tsdf(&morton_arr, &offsets, 5, start, 1.0, &[]);

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
        let tsdf = get_tsdf(&morton_arr, &offsets, trunc, start, 1.0, &[]);

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
        let tsdf = get_tsdf(&morton_arr, &offsets, 5, start, 1.0, &cameras);

        let d_inside = tsdf.get_distance(11.0, 11.0, 11.0);
        assert!(
            d_inside < 0.0,
            "point inside cube shell should be negative, got {}",
            d_inside
        );
    }

    #[test]
    fn surface_normal_points_outward() {
        let pc = PointCloud::new(vec![Vector3::new(10.0, 10.0, 10.0)]);
        let (morton_arr, offsets, start) = morton(&pc, 1.0);
        let tsdf = get_tsdf(&morton_arr, &offsets, 5, start, 1.0, &[]);

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
    fn aabb_from_points_computes_bounds() {
        let points = vec![
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(-1.0, 5.0, 0.0),
            Vector3::new(4.0, 1.0, 7.0),
        ];
        let aabb = Aabb::from_points(&points);
        assert_eq!(aabb.min.x, -1.0);
        assert_eq!(aabb.min.y, 1.0);
        assert_eq!(aabb.min.z, 0.0);
        assert_eq!(aabb.max.x, 4.0);
        assert_eq!(aabb.max.y, 5.0);
        assert_eq!(aabb.max.z, 7.0);
    }

    #[test]
    fn aabb_inflate_expands_uniformly() {
        let aabb = Aabb {
            min: Vector3::new(0.0, 0.0, 0.0),
            max: Vector3::new(1.0, 1.0, 1.0),
        };
        let mut aabb = aabb;
        aabb.inflate(0.5);
        assert_eq!(aabb.min.x, -0.5);
        assert_eq!(aabb.max.x, 1.5);
        assert_eq!(aabb.min.y, -0.5);
        assert_eq!(aabb.max.z, 1.5);
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
        let tsdf = get_tsdf(&morton_arr, &offsets, 3, start, 1.0, &[]);
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
    fn prune_empty_yields_no_tsdf_points() {
        let pc = PointCloud::new(vec![Vector3::new(100.0, 100.0, 100.0)]);
        let roi = Aabb {
            min: Vector3::new(0.0, 0.0, 0.0),
            max: Vector3::new(1.0, 1.0, 1.0),
        };
        let pruned = prune(&pc, Some(roi));
        assert!(pruned.is_empty());
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

    #[test]
    fn from_xyz_file_loads_sphere() {
        let tmp_path = std::env::temp_dir().join(format!(
            "grasp_preshaping_sphere_{}_{}.xyz",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));

        let content = "# synthetic sphere sample\n\
0.10 0.00 0.00\n\
0.00 0.10 0.00\n\
0.00 0.00 0.10\n\
-0.10 0.00 0.00\n\
0.00 -0.10 0.00\n\
0.00 0.00 -0.10\n";
        std::fs::write(&tmp_path, content).unwrap();

        let pc = PointCloud::from_xyz_file(tmp_path.to_str().unwrap()).unwrap();
        let _ = std::fs::remove_file(&tmp_path);
        assert_eq!(pc.len(), 6, "synthetic xyz fixture should parse six points");
        for p in &pc.points {
            assert!(p.x.is_finite() && p.y.is_finite() && p.z.is_finite());
        }
    }

    #[test]
    fn from_xyz_file_missing_file_returns_error() {
        let result = PointCloud::from_xyz_file("/nonexistent/path/input.xyz");
        assert!(result.is_err());
    }

    #[test]
    fn demo_sphere_generates_points() {
        let pc = PointCloud::demo_sphere(Vector3::new(0.1, 0.2, 0.3), 0.05, 100);
        assert_eq!(pc.len(), 100);
        let center = pc.points.iter().fold(Vector3::zeros(), |acc, p| acc + p) / pc.len() as f32;
        assert!((center.x - 0.1).abs() < 0.01, "center x = {}", center.x);
        assert!((center.y - 0.2).abs() < 0.01, "center y = {}", center.y);
        assert!((center.z - 0.3).abs() < 0.01, "center z = {}", center.z);
    }
}
