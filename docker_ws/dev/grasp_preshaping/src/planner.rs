use crate::lut_helper::{Contact, FingerLUT};
use crate::pointcloud_helper::Tsdf;
use nalgebra::{Matrix4, Vector3};

const BINARY_SEARCH_TOL: f64 = 0.01;

#[derive(Debug, Clone)]
pub struct GraspScoreResult {
    pub closure_amount: f64,
    pub alignment_score: f64,
    pub force_closure_score: f64,
    pub found_collision: bool,
}

impl GraspScoreResult {
    pub fn combined_score(&self, weights: &GraspWeights, sample_probability: f64) -> f64 {
        let denom = weights.w_probability + weights.w_alignment + weights.w_force_closure;
        if denom.abs() < 1e-12 {
            return 0.0;
        }
        (weights.w_probability * sample_probability
            + weights.w_alignment * self.alignment_score
            + weights.w_force_closure * self.force_closure_score)
            / denom
    }
}

#[derive(Debug, Clone, Copy)]
pub struct GraspWeights {
    pub w_probability: f64,
    pub w_alignment: f64,
    pub w_force_closure: f64,
}

impl GraspWeights {
    pub fn cylindrical() -> Self {
        Self {
            w_probability: 1.0,
            w_alignment: 1.0,
            w_force_closure: 1.0,
        }
    }

    pub fn pinch() -> Self {
        Self {
            w_probability: 1.0,
            w_alignment: 1.0,
            w_force_closure: 1.0,
        }
    }

    pub fn lateral() -> Self {
        Self {
            w_probability: 1.0,
            w_alignment: 1.0,
            w_force_closure: 1.0,
        }
    }
}

struct ActiveContact {
    surface_normal: Vector3<f32>,
    force_direction: Vector3<f64>,
}

#[derive(Clone, Copy)]
enum Flex {
    Coupled,
    Locked(usize),
}

struct SweepPoint {
    contact: Contact,
    flex: Flex,
}

struct GraspSpec {
    sweep_points: Vec<SweepPoint>,
    score_contacts: Vec<Contact>,
}

pub fn score_cylindrical(
    lut: &FingerLUT,
    tsdf: &Tsdf,
    base_transform: &Matrix4<f64>,
    collision_tol: f32,
) -> GraspScoreResult {
    score_grasp(
        lut,
        tsdf,
        base_transform,
        collision_tol,
        &cylindrical_spec(),
    )
}

pub fn score_pinch(
    lut: &FingerLUT,
    tsdf: &Tsdf,
    base_transform: &Matrix4<f64>,
    collision_tol: f32,
) -> GraspScoreResult {
    score_grasp(lut, tsdf, base_transform, collision_tol, &pinch_spec())
}

pub fn score_lateral(
    lut: &FingerLUT,
    tsdf: &Tsdf,
    base_transform: &Matrix4<f64>,
    collision_tol: f32,
) -> GraspScoreResult {
    score_grasp(lut, tsdf, base_transform, collision_tol, &lateral_spec())
}

fn cylindrical_spec() -> GraspSpec {
    let c = |contact: Contact| SweepPoint {
        contact,
        flex: Flex::Coupled,
    };
    let l = |contact: Contact| SweepPoint {
        contact,
        flex: Flex::Locked(0),
    };

    GraspSpec {
        sweep_points: vec![
            c(Contact::ThumbAbdPip),
            c(Contact::ThumbAbdDip),
            c(Contact::ThumbAbdTip),
            c(Contact::IndexMcp),
            c(Contact::IndexMcpSide),
            c(Contact::IndexDip),
            c(Contact::IndexDipSide),
            c(Contact::IndexPip),
            c(Contact::IndexPipSide),
            c(Contact::IndexTip),
            c(Contact::IndexTipSide),
            c(Contact::MiddleMcp),
            c(Contact::MiddlePip),
            c(Contact::MiddleDip),
            c(Contact::MiddleTip),
            c(Contact::RingDip),
            c(Contact::RingPip),
            c(Contact::RingTip),
            c(Contact::LittleDip),
            c(Contact::LittlePip),
            c(Contact::LittleTip),
            l(Contact::PalmProxUlna),
            l(Contact::PalmProxRadi),
            l(Contact::PalmDistUlna),
            l(Contact::PalmDistRadi),
        ],
        score_contacts: vec![
            Contact::ThumbAbdPip,
            Contact::ThumbAbdDip,
            Contact::ThumbAbdTip,
            Contact::IndexMcp,
            Contact::IndexDip,
            Contact::IndexPip,
            Contact::IndexTip,
            Contact::MiddleMcp,
            Contact::MiddlePip,
            Contact::MiddleDip,
            Contact::MiddleTip,
            Contact::RingDip,
            Contact::RingPip,
            Contact::RingTip,
            Contact::LittleDip,
            Contact::LittlePip,
            Contact::LittleTip,
            Contact::PalmProxUlna,
            Contact::PalmProxRadi,
            Contact::PalmDistUlna,
            Contact::PalmDistRadi,
        ],
    }
}

fn pinch_spec() -> GraspSpec {
    let c = |contact: Contact| SweepPoint {
        contact,
        flex: Flex::Coupled,
    };
    let l = |contact: Contact| SweepPoint {
        contact,
        flex: Flex::Locked(0),
    };

    GraspSpec {
        sweep_points: vec![
            c(Contact::ThumbAbdPip),
            c(Contact::ThumbAbdDip),
            c(Contact::ThumbAbdTip),
            c(Contact::IndexMcp),
            c(Contact::IndexMcpSide),
            c(Contact::IndexDip),
            c(Contact::IndexDipSide),
            c(Contact::IndexPip),
            c(Contact::IndexPipSide),
            c(Contact::IndexTip),
            c(Contact::IndexTipSide),
            l(Contact::MiddleMcp),
            l(Contact::MiddlePip),
            l(Contact::MiddleDip),
            l(Contact::MiddleTip),
            l(Contact::RingDip),
            l(Contact::RingPip),
            l(Contact::RingTip),
            l(Contact::LittleDip),
            l(Contact::LittlePip),
            l(Contact::LittleTip),
            l(Contact::PalmProxUlna),
            l(Contact::PalmProxRadi),
            l(Contact::PalmDistUlna),
            l(Contact::PalmDistRadi),
        ],
        score_contacts: vec![Contact::ThumbAbdTip, Contact::IndexTip],
    }
}

fn lateral_spec() -> GraspSpec {
    let c = |contact: Contact| SweepPoint {
        contact,
        flex: Flex::Coupled,
    };
    let l = |contact: Contact| SweepPoint {
        contact,
        flex: Flex::Locked(0),
    };

    GraspSpec {
        sweep_points: vec![
            c(Contact::ThumbAddPip),
            c(Contact::ThumbAddDip),
            c(Contact::ThumbAddTip),
            c(Contact::IndexMcp),
            c(Contact::IndexMcpSide),
            c(Contact::IndexDip),
            c(Contact::IndexDipSide),
            c(Contact::IndexPip),
            c(Contact::IndexPipSide),
            c(Contact::IndexTip),
            c(Contact::IndexTipSide),
            l(Contact::MiddleMcp),
            l(Contact::MiddlePip),
            l(Contact::MiddleDip),
            l(Contact::MiddleTip),
            l(Contact::RingDip),
            l(Contact::RingPip),
            l(Contact::RingTip),
            l(Contact::LittleDip),
            l(Contact::LittlePip),
            l(Contact::LittleTip),
            l(Contact::PalmProxUlna),
            l(Contact::PalmProxRadi),
            l(Contact::PalmDistUlna),
            l(Contact::PalmDistRadi),
        ],
        score_contacts: vec![
            Contact::ThumbAddTip,
            Contact::IndexMcpSide,
            Contact::IndexDipSide,
            Contact::IndexPipSide,
            Contact::IndexTipSide,
        ],
    }
}

fn score_grasp(
    lut: &FingerLUT,
    tsdf: &Tsdf,
    base_transform: &Matrix4<f64>,
    collision_tol: f32,
    spec: &GraspSpec,
) -> GraspScoreResult {
    match sweep_for_collision(lut, tsdf, base_transform, &spec.sweep_points, collision_tol) {
        None => GraspScoreResult {
            closure_amount: 0.0,
            alignment_score: 0.0,
            force_closure_score: 0.0,
            found_collision: false,
        },
        Some(0) => GraspScoreResult {
            closure_amount: 0.0,
            alignment_score: 0.0,
            force_closure_score: 0.0,
            found_collision: true,
        },
        Some(coll_sample) => {
            let lo_ctrl = lut.get_control(coll_sample - 1);
            let hi_ctrl = lut.get_control(coll_sample);
            let (lo, hi) = refine_binary(
                lut,
                tsdf,
                base_transform,
                &spec.sweep_points,
                collision_tol,
                lo_ctrl,
                hi_ctrl,
            );
            let active = find_active_contacts(
                lut,
                tsdf,
                base_transform,
                &spec.score_contacts,
                lo,
                hi,
                collision_tol,
            );
            GraspScoreResult {
                closure_amount: lo,
                alignment_score: compute_alignment(&active),
                force_closure_score: compute_force_closure(&active),
                found_collision: true,
            }
        }
    }
}

fn sweep_for_collision(
    lut: &FingerLUT,
    tsdf: &Tsdf,
    base: &Matrix4<f64>,
    sweep_points: &[SweepPoint],
    collision_tol: f32,
) -> Option<usize> {
    let resolution = lut.get_resolution();

    for sp in sweep_points {
        if let Flex::Locked(locked_s) = sp.flex {
            let p = pos_at_sample(lut, sp.contact, locked_s, base);
            if tsdf.get_distance(p.x, p.y, p.z) < collision_tol {
                return Some(0);
            }
        }
    }

    for sample in 0..resolution {
        for sp in sweep_points {
            if let Flex::Coupled = sp.flex {
                let p = pos_at_sample(lut, sp.contact, sample, base);
                if tsdf.get_distance(p.x, p.y, p.z) < collision_tol {
                    return Some(sample);
                }
            }
        }
    }

    None
}

fn refine_binary(
    lut: &FingerLUT,
    tsdf: &Tsdf,
    base: &Matrix4<f64>,
    sweep_points: &[SweepPoint],
    collision_tol: f32,
    mut lo: f64,
    mut hi: f64,
) -> (f64, f64) {
    while hi - lo > BINARY_SEARCH_TOL {
        let mid = (lo + hi) * 0.5;
        if collides_at_control(lut, tsdf, base, sweep_points, mid, collision_tol) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    (lo, hi)
}

fn collides_at_control(
    lut: &FingerLUT,
    tsdf: &Tsdf,
    base: &Matrix4<f64>,
    sweep_points: &[SweepPoint],
    control: f64,
    collision_tol: f32,
) -> bool {
    for sp in sweep_points {
        let p = match sp.flex {
            Flex::Coupled => pos_at_control(lut, sp.contact, control, base),
            Flex::Locked(locked_s) => pos_at_sample(lut, sp.contact, locked_s, base),
        };
        if tsdf.get_distance(p.x, p.y, p.z) < collision_tol {
            return true;
        }
    }
    false
}

fn find_active_contacts(
    lut: &FingerLUT,
    tsdf: &Tsdf,
    base: &Matrix4<f64>,
    score_contacts: &[Contact],
    lo_ctrl: f64,
    hi_ctrl: f64,
    collision_tol: f32,
) -> Vec<ActiveContact> {
    let mut active = Vec::new();
    let threshold = collision_tol * 2.0;

    for &contact in score_contacts {
        let p_hi = pos_at_control(lut, contact, hi_ctrl, base);
        let dist = tsdf.get_distance(p_hi.x, p_hi.y, p_hi.z);
        if dist >= threshold {
            continue;
        }

        let normal = tsdf.get_surface_normal(p_hi.x, p_hi.y, p_hi.z);
        let p_lo = pos_at_control(lut, contact, lo_ctrl, base);

        let fd = Vector3::new(
            p_hi.x as f64 - p_lo.x as f64,
            p_hi.y as f64 - p_lo.y as f64,
            p_hi.z as f64 - p_lo.z as f64,
        );
        let norm = fd.norm();
        let force_dir = if norm > 1e-10 {
            fd / norm
        } else {
            Vector3::zeros()
        };

        active.push(ActiveContact {
            surface_normal: normal,
            force_direction: force_dir,
        });
    }

    active
}

fn compute_alignment(contacts: &[ActiveContact]) -> f64 {
    let mut sum = 0.0;
    let mut count = 0usize;

    for c in contacts {
        if c.force_direction.norm() < 0.5 {
            continue;
        }
        let sn = Vector3::new(
            c.surface_normal.x as f64,
            c.surface_normal.y as f64,
            c.surface_normal.z as f64,
        );
        sum += (-sn.dot(&c.force_direction)).max(0.0);
        count += 1;
    }

    if count == 0 {
        0.0
    } else {
        (sum / count as f64).clamp(0.0, 1.0)
    }
}

fn compute_force_closure(contacts: &[ActiveContact]) -> f64 {
    if contacts.is_empty() {
        return 0.0;
    }

    let sum: Vector3<f64> = contacts
        .iter()
        .map(|c| {
            Vector3::new(
                c.surface_normal.x as f64,
                c.surface_normal.y as f64,
                c.surface_normal.z as f64,
            )
        })
        .sum();

    let centroid = sum / contacts.len() as f64;
    (1.0 - centroid.norm()).clamp(0.0, 1.0)
}

fn pos_at_sample(
    lut: &FingerLUT,
    contact: Contact,
    sample: usize,
    base: &Matrix4<f64>,
) -> Vector3<f32> {
    let m = base * lut.get_se_transform_sample(contact, sample);
    Vector3::new(m[(0, 3)] as f32, m[(1, 3)] as f32, m[(2, 3)] as f32)
}

fn pos_at_control(
    lut: &FingerLUT,
    contact: Contact,
    control: f64,
    base: &Matrix4<f64>,
) -> Vector3<f32> {
    let m = base * lut.get_se_transform(contact, control);
    Vector3::new(m[(0, 3)] as f32, m[(1, 3)] as f32, m[(2, 3)] as f32)
}
