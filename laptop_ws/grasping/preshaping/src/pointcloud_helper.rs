use nalgebra::{Matrix4, Vector3};
use std::cmp::Ordering;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

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

    #[cfg(test)]
    pub fn points(&self) -> &[Vector3<f64>] {
        &self.points
    }
}

#[derive(Debug, Clone, Copy)]
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

        Self {
            cloud,
            sorted_x_indices,
            sorted_x_values,
        }
    }

    pub fn nearest_distance(&self, query: &ProximityQuery) -> NearestDistanceResult {
        let world_tip = compose_tip_position(&query.base_transform, &query.finger_transform);

        let mut best_sq: Option<f64> = None;
        let mut candidates_checked = 0usize;

        match query.mask {
            Some(mask) => {
                let left = lower_bound(&self.sorted_x_values, mask.min.x);
                let right = upper_bound(&self.sorted_x_values, mask.max.x);

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
    fn xyz_reader_ignores_comments_and_blank_lines() {
        let raw = "\n# header\n0 0 0\n\n1 2 3\n";
        let reader = BufReader::new(raw.as_bytes());
        let cloud = PointCloud::from_xyz_reader(reader).expect("xyz parsing should succeed");
        assert_eq!(cloud.points().len(), 2);
    }
}
