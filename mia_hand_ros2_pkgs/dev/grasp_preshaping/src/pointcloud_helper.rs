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
            Vector3::new(10.0, 10.0, 12.0),
            Vector3::new(12.0, 12.0, 10.0),
            Vector3::new(12.0, 10.0, 12.0),
            Vector3::new(10.0, 12.0, 12.0),
            Vector3::new(12.0, 12.0, 12.0),
        ]);
        let cameras = vec![
            Camera {
                position: Vector3::new(11.0, 11.0, 8.0),
            },
            Camera {
                position: Vector3::new(11.0, 8.0, 11.0),
            },
            Camera {
                position: Vector3::new(8.0, 11.0, 11.0),
            },
        ];
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
}
