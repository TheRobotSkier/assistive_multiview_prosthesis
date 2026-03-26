use nalgebra::{Matrix3, Matrix4, UnitQuaternion, Vector3};
use npyz::npz::NpzArchive;
use std::collections::HashMap;
use std::error::Error;
use std::io::{Read, Seek};
use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FingerType {
    Index,
    Middle,
    Ring,
    Little,
    ThumbFlex,
    ThumbOpposition,
}

#[derive(Debug, Clone)]
pub struct SE3Matrix {
    pub matrix: Matrix4<f64>,
}

impl SE3Matrix {
    pub fn new(matrix: Matrix4<f64>) -> Self {
        Self { matrix }
    }

    pub fn identity() -> Self {
        Self {
            matrix: Matrix4::identity(),
        }
    }

    pub fn rotation(&self) -> UnitQuaternion<f64> {
        let rotation: Matrix3<f64> = self.matrix.fixed_view::<3, 3>(0, 0).into_owned();
        UnitQuaternion::from_matrix(&rotation)
    }

    pub fn translation(&self) -> Vector3<f64> {
        Vector3::new(
            self.matrix[(0, 3)],
            self.matrix[(1, 3)],
            self.matrix[(2, 3)],
        )
    }
}

#[derive(Error, Debug)]
pub enum LutError {
    #[error("Failed to open .npz file: {0}")]
    FileOpenError(String),
    #[error("Array not found: {0}")]
    ArrayNotFound(String),
    #[error("Invalid array shape: {0}")]
    InvalidShape(String),
    #[error("Attribute not found: {0}")]
    AttributeNotFound(String),
    #[error("Transform lookup failed for {finger:?} at sample {sample} (available samples: {available_samples})")]
    TransformLookupFailed {
        finger: FingerType,
        sample: usize,
        available_samples: usize,
    },
}

pub struct FingerLUT {
    resolution: usize,
    transforms: HashMap<FingerType, Vec<SE3Matrix>>,
}

impl FingerLUT {
    const ROTATION_ORTHO_TOL: f64 = 1e-5;
    const ROTATION_DET_TOL: f64 = 1e-5;
    const HOMOGENEOUS_TOL: f64 = 1e-8;

    fn validate_se3_matrix(
        matrix: &Matrix4<f64>,
        finger_name: &str,
        sample_idx: usize,
    ) -> Result<(), LutError> {
        if !matrix.iter().all(|v| v.is_finite()) {
            return Err(LutError::InvalidShape(format!(
                "{} sample {} has non-finite values",
                finger_name, sample_idx
            )));
        }

        let bottom_row = [
            matrix[(3, 0)],
            matrix[(3, 1)],
            matrix[(3, 2)],
            matrix[(3, 3)],
        ];
        if bottom_row[0].abs() > Self::HOMOGENEOUS_TOL
            || bottom_row[1].abs() > Self::HOMOGENEOUS_TOL
            || bottom_row[2].abs() > Self::HOMOGENEOUS_TOL
            || (bottom_row[3] - 1.0).abs() > Self::HOMOGENEOUS_TOL
        {
            return Err(LutError::InvalidShape(format!(
                "{} sample {} has invalid homogeneous row [{:.6}, {:.6}, {:.6}, {:.6}]",
                finger_name, sample_idx, bottom_row[0], bottom_row[1], bottom_row[2], bottom_row[3]
            )));
        }

        let rotation: Matrix3<f64> = matrix.fixed_view::<3, 3>(0, 0).into_owned();
        let ortho_err = (rotation.transpose() * rotation - Matrix3::identity()).norm();
        if ortho_err > Self::ROTATION_ORTHO_TOL {
            return Err(LutError::InvalidShape(format!(
                "{} sample {} rotation is not orthonormal (error {:.6e})",
                finger_name, sample_idx, ortho_err
            )));
        }

        let det = rotation.determinant();
        if (det - 1.0).abs() > Self::ROTATION_DET_TOL {
            return Err(LutError::InvalidShape(format!(
                "{} sample {} rotation determinant {:.6} is not close to +1",
                finger_name, sample_idx, det
            )));
        }

        Ok(())
    }

    fn read_resolution<R: Read + Seek>(npz: &mut NpzArchive<R>) -> Result<usize, LutError> {
        let read_u32 = npz
            .by_name("resolution")
            .map_err(|e| LutError::FileOpenError(e.to_string()))?
            .ok_or_else(|| LutError::AttributeNotFound("resolution".to_string()))?
            .into_vec::<u32>();
        if let Ok(values) = read_u32 {
            if let Some(value) = values.first() {
                return Ok(*value as usize);
            }
        }

        let read_i32 = npz
            .by_name("resolution")
            .map_err(|e| LutError::FileOpenError(e.to_string()))?
            .ok_or_else(|| LutError::AttributeNotFound("resolution".to_string()))?
            .into_vec::<i32>();
        if let Ok(values) = read_i32 {
            if let Some(value) = values.first() {
                return Ok((*value).max(0) as usize);
            }
        }

        let read_i8 = npz
            .by_name("resolution")
            .map_err(|e| LutError::FileOpenError(e.to_string()))?
            .ok_or_else(|| LutError::AttributeNotFound("resolution".to_string()))?
            .into_vec::<i8>();
        if let Ok(values) = read_i8 {
            if let Some(value) = values.first() {
                return Ok((*value).max(0) as usize);
            }
        }

        Err(LutError::InvalidShape(
            "Failed to read resolution as u32, i32, or i8".to_string(),
        ))
    }

    fn read_transform_data<R: Read + Seek>(
        npz: &mut NpzArchive<R>,
        array_name: &str,
    ) -> Result<Option<Vec<f64>>, LutError> {
        let array = match npz
            .by_name(array_name)
            .map_err(|e| LutError::FileOpenError(e.to_string()))?
        {
            Some(array) => array,
            None => return Ok(None),
        };

        if let Ok(values) = array.into_vec::<f64>() {
            return Ok(Some(values));
        }

        let array_f32 = npz
            .by_name(array_name)
            .map_err(|e| LutError::FileOpenError(e.to_string()))?
            .ok_or_else(|| LutError::ArrayNotFound(array_name.to_string()))?;
        let values_f32: Vec<f32> = array_f32.into_vec::<f32>().map_err(|e| {
            LutError::InvalidShape(format!("Failed to read {} as f32: {}", array_name, e))
        })?;

        Ok(Some(values_f32.into_iter().map(|v| v as f64).collect()))
    }

    pub fn load(path: &str) -> Result<Self, Box<dyn Error>> {
        let mut npz = NpzArchive::open(path).map_err(|e| LutError::FileOpenError(e.to_string()))?;

        // Read resolution
        let resolution_from_file = Self::read_resolution(&mut npz).ok();

        let mut transforms = HashMap::new();

        let finger_types = vec![
            (FingerType::Index, "index_flex"),
            (FingerType::Middle, "middle"),
            (FingerType::Ring, "ring"),
            (FingerType::Little, "little"),
            (FingerType::ThumbFlex, "thumb_flex"),
            (FingerType::ThumbOpposition, "thumb_opposition"),
        ];

        for (finger_type, array_name) in finger_types {
            if let Some(data) = Self::read_transform_data(&mut npz, array_name)? {
                if data.len() % 16 != 0 {
                    return Err(LutError::InvalidShape(format!(
                        "{} has {} values, which is not divisible by 16",
                        array_name,
                        data.len()
                    ))
                    .into());
                }

                // Data should be (resolution, 4, 4) flattened
                let num_samples = data.len() / 16;

                if let Some(expected_resolution) = resolution_from_file {
                    if num_samples != expected_resolution {
                        return Err(LutError::InvalidShape(format!(
                            "{} has {} samples, expected {} from resolution",
                            array_name, num_samples, expected_resolution
                        ))
                        .into());
                    }
                }

                let mut finger_transforms = Vec::with_capacity(num_samples);

                for i in 0..num_samples {
                    let start = i * 16;
                    let mut matrix = Matrix4::zeros();
                    for row in 0..4 {
                        for col in 0..4 {
                            matrix[(row, col)] = data[start + row * 4 + col];
                        }
                    }

                    Self::validate_se3_matrix(&matrix, array_name, i)?;
                    finger_transforms.push(SE3Matrix::new(matrix));
                }

                transforms.insert(finger_type, finger_transforms);
            }
        }

        let inferred_resolution = transforms.values().next().map(|v| v.len()).unwrap_or(0);
        let resolution = resolution_from_file.unwrap_or(inferred_resolution);

        if resolution == 0 {
            return Err(
                LutError::InvalidShape("No transform data found in LUT".to_string()).into(),
            );
        }

        Ok(Self {
            resolution,
            transforms,
        })
    }

    pub fn get_transform(&self, finger: FingerType, sample: usize) -> Option<SE3Matrix> {
        self.transforms.get(&finger)?.get(sample).cloned()
    }

    pub fn get_transform_result(
        &self,
        finger: FingerType,
        sample: usize,
    ) -> Result<SE3Matrix, LutError> {
        self.get_transform(finger, sample)
            .ok_or_else(|| LutError::TransformLookupFailed {
                finger,
                sample,
                available_samples: self.transforms.get(&finger).map(|v| v.len()).unwrap_or(0),
            })
    }

    pub fn get_resolution(&self) -> usize {
        self.resolution
    }

    pub fn get_available_fingers(&self) -> Vec<FingerType> {
        self.transforms.keys().cloned().collect()
    }

    pub fn interpolate_transform(&self, finger: FingerType, t: f64) -> Option<SE3Matrix> {
        let transforms = self.transforms.get(&finger)?;
        if transforms.is_empty() {
            return None;
        }

        let t = t.clamp(0.0, 1.0);
        let num_samples = transforms.len();

        if num_samples == 1 {
            return Some(transforms[0].clone());
        }

        let float_idx = t * (num_samples - 1) as f64;
        let idx = float_idx.floor() as usize;
        let next_idx = (idx + 1).min(num_samples - 1);
        let alpha = float_idx - idx as f64;

        let t1 = &transforms[idx];
        let t2 = &transforms[next_idx];

        let q1 = t1.rotation();
        let q2 = t2.rotation();
        let q_interp = q1.slerp(&q2, alpha);

        let trans1 = t1.translation();
        let trans2 = t2.translation();
        let trans_interp = trans1.lerp(&trans2, alpha);

        let mut matrix = Matrix4::identity();
        matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(q_interp.to_rotation_matrix().matrix());
        matrix[(0, 3)] = trans_interp[0];
        matrix[(1, 3)] = trans_interp[1];
        matrix[(2, 3)] = trans_interp[2];

        Some(SE3Matrix::new(matrix))
    }

    pub fn combine_thumb_transforms(
        &self,
        flex_sample: usize,
        opp_sample: usize,
    ) -> Result<SE3Matrix, LutError> {
        let flex_transform = self.get_transform_result(FingerType::ThumbFlex, flex_sample)?;
        let opp_transform = self.get_transform_result(FingerType::ThumbOpposition, opp_sample)?;

        let combined = flex_transform.matrix * opp_transform.matrix;
        Ok(SE3Matrix::new(combined))
    }
}
