use nalgebra::{Matrix4, Vector3};
use serde::Deserialize;
use std::cmp::Ordering;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;
use std::sync::RwLock;

#[derive(Debug, Clone)]
pub struct PointCloud {
    points: Vec<Vector3<f64>>,
}

impl PointCloud {
    pub fn new(points: Vec<Vector3<f64>>) -> Self {
        Self { points }
    }

    pub fn from_xyz_file(path: impl AsRef<Path>) -> Result<Self, String> {
        let file = File::open(path.as_ref()).map_err(|e| {
            format!(
                "Failed to open .xyz file {}: {}",
                path.as_ref().display(),
                e
            )
        })?;
        let reader = BufReader::new(file);
        Self::from_xyz_reader(reader)
    }

    pub fn from_xyz_reader<R: BufRead>(reader: R) -> Result<Self, String> {
        let mut points = Vec::new();

        for (line_idx, line) in reader.lines().enumerate() {
            let line = line.map_err(|e| format!("Failed to read line {}: {}", line_idx + 1, e))?;
            let trimmed = line.trim();

            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }

            let mut parts = trimmed.split_whitespace();
            let x = parts
                .next()
                .ok_or_else(|| format!("Missing x value at line {}", line_idx + 1))?
                .parse::<f64>()
                .map_err(|e| format!("Invalid x value at line {}: {}", line_idx + 1, e))?;
            let y = parts
                .next()
                .ok_or_else(|| format!("Missing y value at line {}", line_idx + 1))?
                .parse::<f64>()
                .map_err(|e| format!("Invalid y value at line {}: {}", line_idx + 1, e))?;
            let z = parts
                .next()
                .ok_or_else(|| format!("Missing z value at line {}", line_idx + 1))?
                .parse::<f64>()
                .map_err(|e| format!("Invalid z value at line {}: {}", line_idx + 1, e))?;

            points.push(Vector3::new(x, y, z));
        }

        Ok(Self::new(points))
    }

    pub fn from_pointcloud2_yaml(yaml: &str) -> Result<Self, String> {
        let msg: PointCloud2Yaml = serde_yaml::from_str(yaml)
            .map_err(|e| format!("Failed to parse PointCloud2 YAML: {}", e))?;

        if msg.point_step == 0 {
            return Err("PointCloud2 point_step is zero".to_string());
        }

        let x_field = find_field(&msg.fields, "x")?;
        let y_field = find_field(&msg.fields, "y")?;
        let z_field = find_field(&msg.fields, "z")?;

        let point_count_by_data = msg.data.len() / msg.point_step as usize;
        let point_count_by_dims = (msg.width as usize) * (msg.height as usize);
        let point_count = point_count_by_data.min(point_count_by_dims.max(1));

        let mut points = Vec::with_capacity(point_count);
        for i in 0..point_count {
            let base = i * msg.point_step as usize;
            let x = read_point_field(&msg.data, base, x_field, msg.is_bigendian)?;
            let y = read_point_field(&msg.data, base, y_field, msg.is_bigendian)?;
            let z = read_point_field(&msg.data, base, z_field, msg.is_bigendian)?;
            if x.is_finite() && y.is_finite() && z.is_finite() {
                points.push(Vector3::new(x, y, z));
            }
        }

        Ok(Self::new(points))
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn scaled(&self, factor: f64) -> Self {
        if (factor - 1.0).abs() < f64::EPSILON {
            return self.clone();
        }

        let points = self.points.iter().map(|p| p * factor).collect();
        Self::new(points)
    }

    pub fn transformed(&self, transform: &Matrix4<f64>) -> Self {
        let points = self
            .points
            .iter()
            .map(|p| {
                let x = transform[(0, 0)] * p.x
                    + transform[(0, 1)] * p.y
                    + transform[(0, 2)] * p.z
                    + transform[(0, 3)];
                let y = transform[(1, 0)] * p.x
                    + transform[(1, 1)] * p.y
                    + transform[(1, 2)] * p.z
                    + transform[(1, 3)];
                let z = transform[(2, 0)] * p.x
                    + transform[(2, 1)] * p.y
                    + transform[(2, 2)] * p.z
                    + transform[(2, 3)];
                Vector3::new(x, y, z)
            })
            .collect();
        Self::new(points)
    }

    #[cfg(test)]
    pub fn points(&self) -> &[Vector3<f64>] {
        &self.points
    }
}

#[derive(Debug, Deserialize)]
struct PointCloud2Yaml {
    height: u32,
    width: u32,
    fields: Vec<PointFieldYaml>,
    is_bigendian: bool,
    point_step: u32,
    data: Vec<u8>,
}

#[derive(Debug, Deserialize)]
struct PointFieldYaml {
    name: String,
    offset: u32,
    datatype: u8,
}

fn find_field<'a>(fields: &'a [PointFieldYaml], name: &str) -> Result<&'a PointFieldYaml, String> {
    fields
        .iter()
        .find(|f| f.name == name)
        .ok_or_else(|| format!("PointCloud2 field '{}' not found", name))
}

fn read_point_field(
    data: &[u8],
    point_base: usize,
    field: &PointFieldYaml,
    is_bigendian: bool,
) -> Result<f64, String> {
    let off = point_base + field.offset as usize;
    match field.datatype {
        // sensor_msgs/PointField FLOAT32
        7 => {
            let end = off + 4;
            if end > data.len() {
                return Err("PointCloud2 FLOAT32 field exceeds buffer".to_string());
            }
            let mut bytes = [0_u8; 4];
            bytes.copy_from_slice(&data[off..end]);
            let value = if is_bigendian {
                f32::from_be_bytes(bytes)
            } else {
                f32::from_le_bytes(bytes)
            };
            Ok(value as f64)
        }
        // sensor_msgs/PointField FLOAT64
        8 => {
            let end = off + 8;
            if end > data.len() {
                return Err("PointCloud2 FLOAT64 field exceeds buffer".to_string());
            }
            let mut bytes = [0_u8; 8];
            bytes.copy_from_slice(&data[off..end]);
            let value = if is_bigendian {
                f64::from_be_bytes(bytes)
            } else {
                f64::from_le_bytes(bytes)
            };
            Ok(value)
        }
        other => Err(format!(
            "Unsupported PointCloud2 datatype {} for field {}",
            other, field.name
        )),
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AabbMask {
    pub min: Vector3<f64>,
    pub max: Vector3<f64>,
}

impl AabbMask {
    pub fn contains(&self, p: &Vector3<f64>) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }
}

#[derive(Debug, Clone)]
pub struct ProximityQuery {
    pub base_transform: Matrix4<f64>,
    pub finger_transform: Matrix4<f64>,
    pub mask: Option<AabbMask>,
}

#[derive(Debug, Clone)]
pub struct NearestDistanceResult {
    pub world_tip: Vector3<f64>,
    pub nearest_distance: Option<f64>,
    pub candidates_checked: usize,
}

pub struct PointCloudProximityChecker {
    cloud: PointCloud,
    sorted_x_indices: Vec<usize>,
    sorted_x_values: Vec<f64>,
    full_cloud_aabb: Option<AabbMask>,
    cached_user_mask_aabb: RwLock<Option<(AabbMask, Option<AabbMask>)>>,
}

impl PointCloudProximityChecker {
    pub fn new(cloud: PointCloud) -> Self {
        let mut sorted_x_indices: Vec<usize> = (0..cloud.points.len()).collect();
        sorted_x_indices.sort_by(|a, b| {
            cloud.points[*a]
                .x
                .partial_cmp(&cloud.points[*b].x)
                .unwrap_or(Ordering::Equal)
        });
        let sorted_x_values = sorted_x_indices
            .iter()
            .map(|idx| cloud.points[*idx].x)
            .collect();
        let full_cloud_aabb = compute_tight_aabb(cloud.points.iter());

        Self {
            cloud,
            sorted_x_indices,
            sorted_x_values,
            full_cloud_aabb,
            cached_user_mask_aabb: RwLock::new(None),
        }
    }

    pub fn nearest_distance(&self, query: &ProximityQuery) -> NearestDistanceResult {
        let world_tip = compose_tip_position(&query.base_transform, &query.finger_transform);

        let effective_aabb = self.effective_aabb(query.mask);

        let Some(effective_aabb) = effective_aabb else {
            return NearestDistanceResult {
                world_tip,
                nearest_distance: None,
                candidates_checked: 0,
            };
        };

        /*if !effective_aabb.contains(&world_tip) {
            return NearestDistanceResult {
                world_tip,
                nearest_distance: None,
                candidates_checked: 0,
            };
        }*/

        let mut best_sq: Option<f64> = None;
        let mut candidates_checked = 0usize;

        match query.mask {
            Some(mask) => {
                let left = lower_bound(&self.sorted_x_values, effective_aabb.min.x);
                let right = upper_bound(&self.sorted_x_values, effective_aabb.max.x);

                for sorted_pos in left..right {
                    let point_idx = self.sorted_x_indices[sorted_pos];
                    let point = &self.cloud.points[point_idx];

                    if !mask.contains(point) {
                        continue;
                    }

                    candidates_checked += 1;
                    let sq = (point - world_tip).norm_squared();
                    best_sq = Some(best_sq.map_or(sq, |current| current.min(sq)));
                }
            }
            None => {
                for point in &self.cloud.points {
                    candidates_checked += 1;
                    let sq = (point - world_tip).norm_squared();
                    best_sq = Some(best_sq.map_or(sq, |current| current.min(sq)));
                }
            }
        }

        NearestDistanceResult {
            world_tip,
            nearest_distance: best_sq.map(f64::sqrt),
            candidates_checked,
        }
    }

    fn effective_aabb(&self, mask: Option<AabbMask>) -> Option<AabbMask> {
        let Some(mask) = mask else {
            return self.full_cloud_aabb;
        };

        {
            let cached = self
                .cached_user_mask_aabb
                .read()
                .expect("aabb cache lock poisoned");
            if let Some((cached_mask, cached_aabb)) = *cached {
                if cached_mask == mask {
                    return cached_aabb;
                }
            }
        }

        let computed = self.tight_aabb_inside_mask(&mask);
        let mut cached = self
            .cached_user_mask_aabb
            .write()
            .expect("aabb cache lock poisoned");
        *cached = Some((mask, computed));
        computed
    }

    fn tight_aabb_inside_mask(&self, mask: &AabbMask) -> Option<AabbMask> {
        let left = lower_bound(&self.sorted_x_values, mask.min.x);
        let right = upper_bound(&self.sorted_x_values, mask.max.x);
        compute_tight_aabb(
            self.sorted_x_indices[left..right]
                .iter()
                .map(|idx| &self.cloud.points[*idx])
                .filter(|point| mask.contains(point)),
        )
    }
}

fn compute_tight_aabb<'a>(points: impl Iterator<Item = &'a Vector3<f64>>) -> Option<AabbMask> {
    let mut iter = points;
    let first = iter.next()?;

    let mut min = *first;
    let mut max = *first;
    for point in iter {
        min.x = min.x.min(point.x);
        min.y = min.y.min(point.y);
        min.z = min.z.min(point.z);
        max.x = max.x.max(point.x);
        max.y = max.y.max(point.y);
        max.z = max.z.max(point.z);
    }

    Some(AabbMask { min, max })
}

pub fn compose_tip_position(
    base_transform: &Matrix4<f64>,
    finger_transform: &Matrix4<f64>,
) -> Vector3<f64> {
    let world_tip_tf = base_transform * finger_transform;
    Vector3::new(
        world_tip_tf[(0, 3)],
        world_tip_tf[(1, 3)],
        world_tip_tf[(2, 3)],
    )
}

fn lower_bound(values: &[f64], needle: f64) -> usize {
    let mut lo = 0usize;
    let mut hi = values.len();
    while lo < hi {
        let mid = lo + (hi - lo) / 2;
        if values[mid] < needle {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    lo
}

fn upper_bound(values: &[f64], needle: f64) -> usize {
    let mut lo = 0usize;
    let mut hi = values.len();
    while lo < hi {
        let mid = lo + (hi - lo) / 2;
        if values[mid] <= needle {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    lo
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t_xyz(x: f64, y: f64, z: f64) -> Matrix4<f64> {
        let mut m = Matrix4::identity();
        m[(0, 3)] = x;
        m[(1, 3)] = y;
        m[(2, 3)] = z;
        m
    }

    #[test]
    fn compose_tip_position_uses_base_times_finger() {
        let base = t_xyz(1.0, 2.0, 3.0);
        let finger = t_xyz(0.5, -1.0, 2.0);
        let world_tip = compose_tip_position(&base, &finger);

        assert!((world_tip.x - 1.5).abs() < 1e-12);
        assert!((world_tip.y - 1.0).abs() < 1e-12);
        assert!((world_tip.z - 5.0).abs() < 1e-12);
    }

    #[test]
    fn nearest_distance_without_mask_scans_all_points() {
        let cloud = PointCloud::new(vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(3.0, 0.0, 0.0),
        ]);
        let checker = PointCloudProximityChecker::new(cloud);

        let query = ProximityQuery {
            base_transform: Matrix4::identity(),
            finger_transform: t_xyz(2.0, 0.0, 0.0),
            mask: None,
        };

        let result = checker.nearest_distance(&query);
        assert_eq!(result.candidates_checked, 3);
        assert_eq!(result.nearest_distance, Some(1.0));
    }

    #[test]
    fn nearest_distance_with_mask_limits_candidates() {
        let cloud = PointCloud::new(vec![
            Vector3::new(-10.0, 0.0, 0.0),
            Vector3::new(1.5, 0.0, 0.0),
            Vector3::new(2.5, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
        ]);
        let checker = PointCloudProximityChecker::new(cloud);

        let query = ProximityQuery {
            base_transform: Matrix4::identity(),
            finger_transform: t_xyz(2.0, 0.0, 0.0),
            mask: Some(AabbMask {
                min: Vector3::new(1.0, -1.0, -1.0),
                max: Vector3::new(3.0, 1.0, 1.0),
            }),
        };

        let result = checker.nearest_distance(&query);
        assert_eq!(result.candidates_checked, 2);
        assert_eq!(result.nearest_distance, Some(0.5));
    }

    #[test]
    fn nearest_distance_with_mask_can_return_none() {
        let cloud = PointCloud::new(vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
        ]);
        let checker = PointCloudProximityChecker::new(cloud);

        let query = ProximityQuery {
            base_transform: Matrix4::identity(),
            finger_transform: Matrix4::identity(),
            mask: Some(AabbMask {
                min: Vector3::new(5.0, 5.0, 5.0),
                max: Vector3::new(6.0, 6.0, 6.0),
            }),
        };

        let result = checker.nearest_distance(&query);
        assert_eq!(result.candidates_checked, 0);
        assert_eq!(result.nearest_distance, None);
    }

    #[test]
    fn nearest_distance_without_mask_skips_when_tip_outside_cloud_aabb() {
        let cloud = PointCloud::new(vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
        ]);
        let checker = PointCloudProximityChecker::new(cloud);

        let query = ProximityQuery {
            base_transform: Matrix4::identity(),
            finger_transform: t_xyz(10.0, 10.0, 10.0),
            mask: None,
        };

        let result = checker.nearest_distance(&query);
        assert_eq!(result.candidates_checked, 0);
        assert_eq!(result.nearest_distance, None);
    }

    #[test]
    fn nearest_distance_with_mask_uses_tight_subset_for_tip_gate() {
        let cloud = PointCloud::new(vec![
            Vector3::new(10.0, 10.0, 10.0),
            Vector3::new(11.0, 11.0, 11.0),
            Vector3::new(50.0, 50.0, 50.0),
        ]);
        let checker = PointCloudProximityChecker::new(cloud);

        let query = ProximityQuery {
            base_transform: Matrix4::identity(),
            finger_transform: t_xyz(5.0, 5.0, 5.0),
            mask: Some(AabbMask {
                min: Vector3::new(0.0, 0.0, 0.0),
                max: Vector3::new(20.0, 20.0, 20.0),
            }),
        };

        let result = checker.nearest_distance(&query);
        assert_eq!(result.candidates_checked, 0);
        assert_eq!(result.nearest_distance, None);
    }

    #[test]
    fn nearest_distance_tip_on_computed_aabb_boundary_is_included() {
        let cloud = PointCloud::new(vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
        ]);
        let checker = PointCloudProximityChecker::new(cloud);

        let query = ProximityQuery {
            base_transform: Matrix4::identity(),
            finger_transform: t_xyz(0.0, 0.0, 0.0),
            mask: None,
        };

        let result = checker.nearest_distance(&query);
        assert_eq!(result.candidates_checked, 2);
        assert_eq!(result.nearest_distance, Some(0.0));
    }

    #[test]
    fn xyz_reader_ignores_comments_and_blank_lines() {
        let raw = "\n# header\n0 0 0\n\n1 2 3\n";
        let reader = BufReader::new(raw.as_bytes());
        let cloud = PointCloud::from_xyz_reader(reader).expect("xyz parsing should succeed");
        assert_eq!(cloud.points().len(), 2);
    }

    #[test]
    fn pointcloud2_yaml_parses_float32_xyz() {
        // Two points: (1,2,3) and (4,5,6) packed as little-endian float32 xyz.
        let yaml = "height: 1
width: 2
fields:
  - {name: x, offset: 0, datatype: 7}
  - {name: y, offset: 4, datatype: 7}
  - {name: z, offset: 8, datatype: 7}
is_bigendian: false
point_step: 12
data: [0,0,128,63,0,0,0,64,0,0,64,64,0,0,128,64,0,0,160,64,0,0,192,64]
";

        let cloud = PointCloud::from_pointcloud2_yaml(yaml).expect("PointCloud2 parse should work");
        assert_eq!(cloud.points().len(), 2);

        let p0 = cloud.points()[0];
        let p1 = cloud.points()[1];
        assert!((p0.x - 1.0).abs() < 1e-9);
        assert!((p0.y - 2.0).abs() < 1e-9);
        assert!((p0.z - 3.0).abs() < 1e-9);
        assert!((p1.x - 4.0).abs() < 1e-9);
        assert!((p1.y - 5.0).abs() < 1e-9);
        assert!((p1.z - 6.0).abs() < 1e-9);
    }

    #[test]
    fn scaled_multiplies_all_points() {
        let cloud = PointCloud::new(vec![Vector3::new(1.0, -2.0, 3.0), Vector3::new(0.5, 1.0, -1.5)]);
        let scaled = cloud.scaled(0.01);

        let p0 = scaled.points()[0];
        let p1 = scaled.points()[1];
        assert!((p0.x - 0.01).abs() < 1e-12);
        assert!((p0.y + 0.02).abs() < 1e-12);
        assert!((p0.z - 0.03).abs() < 1e-12);
        assert!((p1.x - 0.005).abs() < 1e-12);
        assert!((p1.y - 0.01).abs() < 1e-12);
        assert!((p1.z + 0.015).abs() < 1e-12);
    }

    #[test]
    fn transformed_applies_rigid_translation() {
        let cloud = PointCloud::new(vec![Vector3::new(1.0, -2.0, 3.0), Vector3::new(0.5, 1.0, -1.5)]);
        let mut tf = Matrix4::identity();
        tf[(0, 3)] = -0.1;
        tf[(1, 3)] = 0.2;
        tf[(2, 3)] = 0.3;

        let transformed = cloud.transformed(&tf);

        let p0 = transformed.points()[0];
        let p1 = transformed.points()[1];
        assert!((p0.x - 0.9).abs() < 1e-12);
        assert!((p0.y + 1.8).abs() < 1e-12);
        assert!((p0.z - 3.3).abs() < 1e-12);
        assert!((p1.x - 0.4).abs() < 1e-12);
        assert!((p1.y - 1.2).abs() < 1e-12);
        assert!((p1.z + 1.2).abs() < 1e-12);
    }
}
