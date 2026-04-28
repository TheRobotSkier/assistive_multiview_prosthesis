use nalgebra::{Matrix4, Quaternion, Rotation3, UnitQuaternion, Vector3, Vector4};
use npyz::npz::NpzArchive;
use std::io::{Read, Seek};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DualQuaternion {
    pub real: [f64; 4],
    pub dual: [f64; 4],
}

impl DualQuaternion {
    pub fn to_se3(self) -> Matrix4<f64> {
        let qr = self.normalized_real_quaternion();
        let qd = Quaternion::new(self.dual[0], self.dual[1], self.dual[2], self.dual[3]);
        let t_quat = qd * qr.conjugate() * 2.0;

        let rotation = UnitQuaternion::from_quaternion(qr)
            .to_rotation_matrix()
            .matrix()
            .clone_owned();

        let mut m = Matrix4::identity();
        m.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
        m[(0, 3)] = t_quat.i;
        m[(1, 3)] = t_quat.j;
        m[(2, 3)] = t_quat.k;
        m
    }

    pub fn location(self) -> Vector3<f64> {
        let se3 = self.to_se3();
        Vector3::new(se3[(0, 3)], se3[(1, 3)], se3[(2, 3)])
    }

    fn from_slice(v: &[f64]) -> Self {
        Self {
            real: [v[0], v[1], v[2], v[3]],
            dual: [v[4], v[5], v[6], v[7]],
        }
    }

    fn normalized_real_quaternion(self) -> Quaternion<f64> {
        let q = Quaternion::new(self.real[0], self.real[1], self.real[2], self.real[3]);
        let norm = q.norm();
        if norm > 0.0 {
            q / norm
        } else {
            Quaternion::identity()
        }
    }

    fn lerp(a: Self, b: Self, t: f64) -> Self {
        let mut b_real = b.real;
        let mut b_dual = b.dual;

        // Keep quaternion sign continuity before blending.
        let dot = a.real[0] * b.real[0]
            + a.real[1] * b.real[1]
            + a.real[2] * b.real[2]
            + a.real[3] * b.real[3];
        if dot < 0.0 {
            for i in 0..4 {
                b_real[i] = -b_real[i];
                b_dual[i] = -b_dual[i];
            }
        }

        let mut out_real = [0.0; 4];
        let mut out_dual = [0.0; 4];
        for i in 0..4 {
            out_real[i] = (1.0 - t) * a.real[i] + t * b_real[i];
            out_dual[i] = (1.0 - t) * a.dual[i] + t * b_dual[i];
        }

        let mut dq = Self {
            real: out_real,
            dual: out_dual,
        };
        dq.normalize_in_place();
        dq
    }

    pub fn identity() -> Self {
        Self {
            real: [1.0, 0.0, 0.0, 0.0],
            dual: [0.0, 0.0, 0.0, 0.0],
        }
    }

    pub fn multiply(&self, other: &DualQuaternion) -> DualQuaternion {
        let qr1 = Quaternion::new(self.real[0], self.real[1], self.real[2], self.real[3]);
        let qd1 = Quaternion::new(self.dual[0], self.dual[1], self.dual[2], self.dual[3]);
        let qr2 = Quaternion::new(other.real[0], other.real[1], other.real[2], other.real[3]);
        let qd2 = Quaternion::new(other.dual[0], other.dual[1], other.dual[2], other.dual[3]);

        let qr_out = qr1 * qr2;
        let qd_out = qr1 * qd2 + qd1 * qr2;

        let mut dq = DualQuaternion {
            real: [qr_out.w, qr_out.i, qr_out.j, qr_out.k],
            dual: [qd_out.w, qd_out.i, qd_out.j, qd_out.k],
        };
        dq.normalize_in_place();
        dq
    }

    pub fn from_se3(m: &Matrix4<f64>) -> Self {
        let rot = m.fixed_view::<3, 3>(0, 0);
        let rot3 = Rotation3::from_matrix_unchecked(rot.clone_owned());
        let uq = UnitQuaternion::from_rotation_matrix(&rot3);
        let qr = uq.quaternion();

        let tx = m[(0, 3)];
        let ty = m[(1, 3)];
        let tz = m[(2, 3)];
        let t_pure = Quaternion::new(0.0, tx, ty, tz);
        let qd = t_pure * qr * 0.5;

        Self {
            real: [qr.w, qr.i, qr.j, qr.k],
            dual: [qd.w, qd.i, qd.j, qd.k],
        }
    }

    pub fn transform_point(&self, p: &Vector3<f64>) -> Vector3<f64> {
        let se3 = self.to_se3();
        let homo = se3 * Vector4::new(p.x, p.y, p.z, 1.0);
        Vector3::new(homo.x, homo.y, homo.z)
    }

    fn normalize_in_place(&mut self) {
        let n = (self.real[0] * self.real[0]
            + self.real[1] * self.real[1]
            + self.real[2] * self.real[2]
            + self.real[3] * self.real[3])
            .sqrt();
        if n > 0.0 {
            for i in 0..4 {
                self.real[i] /= n;
                self.dual[i] /= n;
            }
        }
    }

    #[cfg(test)]
    fn from_translation_xyz(x: f64, y: f64, z: f64) -> Self {
        // qr = [1,0,0,0], qd = 0.5 * [0,tx,ty,tz] * qr
        Self {
            real: [1.0, 0.0, 0.0, 0.0],
            dual: [0.0, 0.5 * x, 0.5 * y, 0.5 * z],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Contact {
    IndexPip,
    IndexPipSide,
    IndexMcp,
    IndexMcpSide,
    IndexDip,
    IndexDipSide,
    IndexTip,
    IndexTipSide,
    MiddleMcp,
    MiddleDip,
    MiddlePip,
    MiddleTip,
    RingDip,
    RingPip,
    RingTip,
    LittleDip,
    LittlePip,
    LittleTip,
    ThumbAddPip,
    ThumbAddDip,
    ThumbAddTip,
    ThumbAbdPip,
    ThumbAbdDip,
    ThumbAbdTip,
    PalmProxUlna,
    PalmProxRadi,
    PalmDistUlna,
    PalmDistRadi,
}

// Thumb has two opposing modes: Adduction (lateral key-grip) and Abduction
// (opposition, cylindrical/pinch).  The LUT stores separate tables for each.
#[derive(Debug, Clone, Copy)]
enum ContactTable {
    Index,
    Mrl,
    ThumbAdd,
    ThumbAbd,
    Palm,
}

#[derive(Debug, Clone, Copy)]
struct ContactSpec {
    table: ContactTable,
    index: usize,
}

pub struct FingerLUT {
    resolution: usize,
    index_table: Vec<DualQuaternion>,
    mrl_table: Vec<DualQuaternion>,
    thumb_add_table: Vec<DualQuaternion>,
    thumb_abd_table: Vec<DualQuaternion>,
    palm_table: Vec<DualQuaternion>,
    /// Maximum closure amount before self-collision, per grasp type.
    /// Index 0 = cylindrical, 1 = pinch, 2 = lateral.
    /// Values in [0, 1] normalized closure.
    max_closure_per_grasp_type: [f64; 3],
}

impl FingerLUT {
    const INDEX_CONTACTS: usize = 8;
    const MRL_CONTACTS: usize = 10;
    const THUMB_CONTACTS: usize = 3;
    const PALM_CONTACTS: usize = 4;

    pub fn load(path: &str) -> Self {
        let mut npz = NpzArchive::open(path).expect("failed to open LUT npz file");

        let resolution = Self::read_resolution(&mut npz);
        let index_raw = Self::read_numeric_array(&mut npz, "index_table");
        let mrl_raw = Self::read_numeric_array(&mut npz, "mrl_table");
        let thumb_add_raw = Self::read_numeric_array(&mut npz, "thumb_opp_mode0_table");
        let thumb_abd_raw = Self::read_numeric_array(&mut npz, "thumb_opp_mode1_table");
        let palm_raw = Self::read_numeric_array(&mut npz, "palm_table");

        let index_table =
            Self::decode_table(&index_raw, resolution, Self::INDEX_CONTACTS, "index_table");
        let mrl_table = Self::decode_table(&mrl_raw, resolution, Self::MRL_CONTACTS, "mrl_table");
        let thumb_add_table = Self::decode_table(
            &thumb_add_raw,
            resolution,
            Self::THUMB_CONTACTS,
            "thumb_opp_mode0_table",
        );
        let thumb_abd_table = Self::decode_table(
            &thumb_abd_raw,
            resolution,
            Self::THUMB_CONTACTS,
            "thumb_opp_mode1_table",
        );
        let palm_table = Self::decode_palm_table(&palm_raw, Self::PALM_CONTACTS, "palm_table");

        // Load max closure per grasp type (optional for backward compatibility).
        let max_closure_per_grasp_type = Self::read_max_closure(&mut npz);

        Self {
            resolution,
            index_table,
            mrl_table,
            thumb_add_table,
            thumb_abd_table,
            palm_table,
            max_closure_per_grasp_type,
        }
    }

    pub fn get_se_transform(&self, contact: Contact, control: f64) -> Matrix4<f64> {
        self.get_dq(contact, control).to_se3()
    }

    pub fn get_location(&self, contact: Contact, control: f64) -> Vector3<f64> {
        self.get_dq(contact, control).location()
    }

    pub fn get_dq(&self, contact: Contact, control: f64) -> DualQuaternion {
        let spec = Self::contact_spec(contact);
        if matches!(spec.table, ContactTable::Palm) {
            return self.palm_table[spec.index];
        }

        let clamped = control.clamp(0.0, 1.0);
        if self.resolution <= 1 {
            return self.get_dq_sample(contact, 0);
        }

        let float_idx = clamped * (self.resolution - 1) as f64;
        let i0 = float_idx.floor() as usize;
        let i1 = (i0 + 1).min(self.resolution - 1);
        let alpha = float_idx - i0 as f64;

        let d0 = self.get_dq_sample(contact, i0);
        let d1 = self.get_dq_sample(contact, i1);
        DualQuaternion::lerp(d0, d1, alpha)
    }

    pub fn get_se_transform_sample(&self, contact: Contact, sample: usize) -> Matrix4<f64> {
        self.get_dq_sample(contact, sample).to_se3()
    }

    pub fn get_location_sample(&self, contact: Contact, sample: usize) -> Vector3<f64> {
        self.get_dq_sample(contact, sample).location()
    }

    pub fn get_dq_sample(&self, contact: Contact, sample: usize) -> DualQuaternion {
        let spec = Self::contact_spec(contact);
        match spec.table {
            ContactTable::Index => {
                assert!(
                    sample < self.resolution,
                    "sample out of range for index table"
                );
                self.index_table[sample * Self::INDEX_CONTACTS + spec.index]
            }
            ContactTable::Mrl => {
                assert!(
                    sample < self.resolution,
                    "sample out of range for mrl table"
                );
                self.mrl_table[sample * Self::MRL_CONTACTS + spec.index]
            }
            ContactTable::ThumbAdd => {
                assert!(
                    sample < self.resolution,
                    "sample out of range for thumb add table"
                );
                self.thumb_add_table[sample * Self::THUMB_CONTACTS + spec.index]
            }
            ContactTable::ThumbAbd => {
                assert!(
                    sample < self.resolution,
                    "sample out of range for thumb abd table"
                );
                self.thumb_abd_table[sample * Self::THUMB_CONTACTS + spec.index]
            }
            ContactTable::Palm => self.palm_table[spec.index],
        }
    }

    pub fn get_resolution(&self) -> usize {
        self.resolution
    }

    pub fn get_control(&self, sample: usize) -> f64 {
        assert!(
            sample < self.resolution,
            "sample out of range in get_control"
        );
        if self.resolution <= 1 {
            0.0
        } else {
            sample as f64 / (self.resolution - 1) as f64
        }
    }

    pub fn get_sample(&self, control: f64) -> usize {
        if self.resolution <= 1 {
            return 0;
        }

        let clamped = control.clamp(0.0, 1.0);
        let idx = (clamped * (self.resolution - 1) as f64).round() as usize;
        idx.min(self.resolution - 1)
    }

    /// Returns the maximum closure amount before self-collision for the given grasp type.
    /// `grasp_type_index`: 0 = cylindrical, 1 = pinch, 2 = lateral.
    /// Returns 1.0 if the data is not available in the LUT (backward compat).
    pub fn get_max_closure(&self, grasp_type_index: usize) -> f64 {
        self.max_closure_per_grasp_type
            .get(grasp_type_index)
            .copied()
            .unwrap_or(1.0)
    }

    fn read_resolution<R: Read + Seek>(npz: &mut NpzArchive<R>) -> usize {
        let as_i32 = npz
            .by_name("resolution")
            .expect("failed to access resolution")
            .expect("resolution array missing")
            .into_vec::<i32>();

        if let Ok(v) = as_i32 {
            assert!(!v.is_empty(), "resolution array is empty");
            return v[0].max(1) as usize;
        }

        let as_u32 = npz
            .by_name("resolution")
            .expect("failed to access resolution")
            .expect("resolution array missing")
            .into_vec::<u32>()
            .expect("failed to decode resolution as i32 or u32");
        assert!(!as_u32.is_empty(), "resolution array is empty");
        as_u32[0].max(1) as usize
    }

    fn read_numeric_array<R: Read + Seek>(npz: &mut NpzArchive<R>, name: &str) -> Vec<f64> {
        let arr_f32 = npz
            .by_name(name)
            .unwrap_or_else(|_| panic!("failed to access {}", name))
            .unwrap_or_else(|| panic!("{} array missing", name))
            .into_vec::<f32>();

        if let Ok(v) = arr_f32 {
            return v.into_iter().map(|x| x as f64).collect();
        }

        npz.by_name(name)
            .unwrap_or_else(|_| panic!("failed to access {}", name))
            .unwrap_or_else(|| panic!("{} array missing", name))
            .into_vec::<f64>()
            .unwrap_or_else(|_| panic!("failed to decode {} as f32 or f64", name))
    }

    fn decode_table(
        data: &[f64],
        samples: usize,
        contacts: usize,
        name: &str,
    ) -> Vec<DualQuaternion> {
        let expected = samples * contacts * 8;
        assert_eq!(
            data.len(),
            expected,
            "{} has unexpected length (got {}, expected {})",
            name,
            data.len(),
            expected
        );

        data.chunks_exact(8)
            .map(DualQuaternion::from_slice)
            .collect()
    }

    fn decode_palm_table(data: &[f64], contacts: usize, name: &str) -> Vec<DualQuaternion> {
        let expected = contacts * 8;
        assert_eq!(
            data.len(),
            expected,
            "{} has unexpected length (got {}, expected {})",
            name,
            data.len(),
            expected
        );

        data.chunks_exact(8)
            .map(DualQuaternion::from_slice)
            .collect()
    }

    /// Read max_closure_per_grasp_type from the npz. Returns [1.0, 1.0, 1.0]
    /// if the field is missing (backward compatibility with older LUT files).
    fn read_max_closure<R: Read + Seek>(npz: &mut NpzArchive<R>) -> [f64; 3] {
        let default = [1.0, 1.0, 1.0];

        let arr = match npz.by_name("max_closure_per_grasp_type") {
            Ok(Some(reader)) => reader,
            _ => return default,
        };

        // Try f32 first (as stored by Python).
        if let Ok(vec) = arr.into_vec::<f32>() {
            if vec.len() >= 3 {
                return [vec[0] as f64, vec[1] as f64, vec[2] as f64];
            }
            return default;
        }

        // Try f64.
        let arr2 = match npz.by_name("max_closure_per_grasp_type") {
            Ok(Some(reader)) => reader,
            _ => return default,
        };
        if let Ok(vec) = arr2.into_vec::<f64>() {
            if vec.len() >= 3 {
                return [vec[0], vec[1], vec[2]];
            }
        }

        default
    }

    fn contact_spec(contact: Contact) -> ContactSpec {
        match contact {
            Contact::IndexMcp => ContactSpec {
                table: ContactTable::Index,
                index: 0,
            },
            Contact::IndexMcpSide => ContactSpec {
                table: ContactTable::Index,
                index: 1,
            },
            Contact::IndexDip => ContactSpec {
                table: ContactTable::Index,
                index: 2,
            },
            Contact::IndexDipSide => ContactSpec {
                table: ContactTable::Index,
                index: 3,
            },
            Contact::IndexPip => ContactSpec {
                table: ContactTable::Index,
                index: 4,
            },
            Contact::IndexPipSide => ContactSpec {
                table: ContactTable::Index,
                index: 5,
            },
            Contact::IndexTip => ContactSpec {
                table: ContactTable::Index,
                index: 6,
            },
            Contact::IndexTipSide => ContactSpec {
                table: ContactTable::Index,
                index: 7,
            },
            Contact::MiddleMcp => ContactSpec {
                table: ContactTable::Mrl,
                index: 0,
            },
            Contact::MiddlePip => ContactSpec {
                table: ContactTable::Mrl,
                index: 1,
            },
            Contact::MiddleDip => ContactSpec {
                table: ContactTable::Mrl,
                index: 2,
            },
            Contact::MiddleTip => ContactSpec {
                table: ContactTable::Mrl,
                index: 3,
            },
            Contact::RingDip => ContactSpec {
                table: ContactTable::Mrl,
                index: 4,
            },
            Contact::RingPip => ContactSpec {
                table: ContactTable::Mrl,
                index: 5,
            },
            Contact::RingTip => ContactSpec {
                table: ContactTable::Mrl,
                index: 6,
            },
            Contact::LittleDip => ContactSpec {
                table: ContactTable::Mrl,
                index: 7,
            },
            Contact::LittlePip => ContactSpec {
                table: ContactTable::Mrl,
                index: 8,
            },
            Contact::LittleTip => ContactSpec {
                table: ContactTable::Mrl,
                index: 9,
            },
            Contact::ThumbAddPip => ContactSpec {
                table: ContactTable::ThumbAdd,
                index: 0,
            },
            Contact::ThumbAddDip => ContactSpec {
                table: ContactTable::ThumbAdd,
                index: 1,
            },
            Contact::ThumbAddTip => ContactSpec {
                table: ContactTable::ThumbAdd,
                index: 2,
            },
            Contact::ThumbAbdPip => ContactSpec {
                table: ContactTable::ThumbAbd,
                index: 0,
            },
            Contact::ThumbAbdDip => ContactSpec {
                table: ContactTable::ThumbAbd,
                index: 1,
            },
            Contact::ThumbAbdTip => ContactSpec {
                table: ContactTable::ThumbAbd,
                index: 2,
            },
            Contact::PalmProxUlna => ContactSpec {
                table: ContactTable::Palm,
                index: 0,
            },
            Contact::PalmProxRadi => ContactSpec {
                table: ContactTable::Palm,
                index: 1,
            },
            Contact::PalmDistUlna => ContactSpec {
                table: ContactTable::Palm,
                index: 2,
            },
            Contact::PalmDistRadi => ContactSpec {
                table: ContactTable::Palm,
                index: 3,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Contact, DualQuaternion, FingerLUT};
    use nalgebra::{Matrix4, UnitQuaternion, Vector3, Vector4};

    fn build_test_lut() -> FingerLUT {
        let resolution = 5;

        let mut index_table = Vec::new();
        for s in 0..resolution {
            for c in 0..8 {
                index_table.push(DualQuaternion::from_translation_xyz(
                    s as f64 + c as f64,
                    0.0,
                    0.0,
                ));
            }
        }

        let mut mrl_table = Vec::new();
        for s in 0..resolution {
            for c in 0..10 {
                mrl_table.push(DualQuaternion::from_translation_xyz(
                    100.0 + s as f64 + c as f64,
                    0.0,
                    0.0,
                ));
            }
        }

        let mut thumb_add_table = Vec::new();
        let mut thumb_abd_table = Vec::new();
        for s in 0..resolution {
            for c in 0..3 {
                thumb_add_table.push(DualQuaternion::from_translation_xyz(
                    200.0 + s as f64 + c as f64,
                    0.0,
                    0.0,
                ));
                thumb_abd_table.push(DualQuaternion::from_translation_xyz(
                    300.0 + s as f64 + c as f64,
                    0.0,
                    0.0,
                ));
            }
        }

        let palm_table = vec![
            DualQuaternion::from_translation_xyz(400.0, 0.0, 0.0),
            DualQuaternion::from_translation_xyz(401.0, 0.0, 0.0),
            DualQuaternion::from_translation_xyz(402.0, 0.0, 0.0),
            DualQuaternion::from_translation_xyz(403.0, 0.0, 0.0),
        ];

        FingerLUT {
            resolution,
            index_table,
            mrl_table,
            thumb_add_table,
            thumb_abd_table,
            palm_table,
            max_closure_per_grasp_type: [1.0, 1.0, 1.0],
        }
    }

    #[test]
    fn sample_control_roundtrip_is_stable() {
        let lut = build_test_lut();
        for s in 0..lut.get_resolution() {
            let c = lut.get_control(s);
            let s2 = lut.get_sample(c);
            assert_eq!(s, s2);
        }
    }

    #[test]
    fn control_endpoints_match_sample_endpoints() {
        let lut = build_test_lut();

        let first = lut.get_location(Contact::ThumbAbdTip, 0.0);
        let first_sample = lut.get_location_sample(Contact::ThumbAbdTip, 0);
        assert!((first[0] - first_sample[0]).abs() < 1e-9);

        let last = lut.get_location(Contact::ThumbAbdTip, 1.0);
        let last_sample = lut.get_location_sample(Contact::ThumbAbdTip, lut.get_resolution() - 1);
        assert!((last[0] - last_sample[0]).abs() < 1e-9);
    }

    #[test]
    fn sample_query_returns_finite_values() {
        let lut = build_test_lut();
        let p = lut.get_location_sample(Contact::IndexPipSide, 3);
        assert!(p.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn location_matches_transform_translation() {
        let lut = build_test_lut();
        let p = lut.get_location(Contact::PalmDistUlna, 0.42);
        let m = lut.get_se_transform(Contact::PalmDistUlna, 0.42);
        assert!((p[0] - m[(0, 3)]).abs() < 1e-9);
        assert!((p[1] - m[(1, 3)]).abs() < 1e-9);
        assert!((p[2] - m[(2, 3)]).abs() < 1e-9);
    }

    #[test]
    fn thumb_modes_are_distinct() {
        let lut = build_test_lut();
        let add = lut.get_location_sample(Contact::ThumbAddTip, 2);
        let abd = lut.get_location_sample(Contact::ThumbAbdTip, 2);
        assert!(abd[0] > add[0]);
    }

    #[test]
    fn identity_multiply_is_noop() {
        let a = DualQuaternion::from_translation_xyz(1.0, 2.0, 3.0);
        let id = DualQuaternion::identity();
        let result = a.multiply(&id);
        let loc = result.location();
        assert!((loc[0] - 1.0).abs() < 1e-9);
        assert!((loc[1] - 2.0).abs() < 1e-9);
        assert!((loc[2] - 3.0).abs() < 1e-9);
    }

    #[test]
    fn multiply_composes_translations() {
        let a = DualQuaternion::from_translation_xyz(1.0, 0.0, 0.0);
        let b = DualQuaternion::from_translation_xyz(0.0, 2.0, 0.0);
        let result = a.multiply(&b);
        let loc = result.location();
        assert!((loc[0] - 1.0).abs() < 1e-9);
        assert!((loc[1] - 2.0).abs() < 1e-9);
        assert!((loc[2] - 0.0).abs() < 1e-9);
    }

    #[test]
    fn from_se3_to_se3_roundtrip() {
        let mut m = Matrix4::identity();
        m[(0, 3)] = 1.5;
        m[(1, 3)] = -2.0;
        m[(2, 3)] = 0.5;
        let rot = UnitQuaternion::from_euler_angles(0.3, 0.5, 0.7);
        m.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(rot.to_rotation_matrix().matrix());

        let dq = DualQuaternion::from_se3(&m);
        let m_back = dq.to_se3();

        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (m[(i, j)] - m_back[(i, j)]).abs() < 1e-9,
                    "mismatch at ({}, {}): {} vs {}",
                    i,
                    j,
                    m[(i, j)],
                    m_back[(i, j)]
                );
            }
        }
    }

    #[test]
    fn transform_point_matches_matrix() {
        let mut m = Matrix4::identity();
        m[(0, 3)] = 1.0;
        m[(1, 3)] = 2.0;
        m[(2, 3)] = 3.0;
        let rot = UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
        m.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(rot.to_rotation_matrix().matrix());

        let dq = DualQuaternion::from_se3(&m);
        let p = Vector3::new(0.5, -0.5, 1.0);

        let result_dq = dq.transform_point(&p);
        let homo = m * Vector4::new(p.x, p.y, p.z, 1.0);
        let result_mat = Vector3::new(homo.x, homo.y, homo.z);

        assert!((result_dq.x - result_mat.x).abs() < 1e-9);
        assert!((result_dq.y - result_mat.y).abs() < 1e-9);
        assert!((result_dq.z - result_mat.z).abs() < 1e-9);
    }
}
