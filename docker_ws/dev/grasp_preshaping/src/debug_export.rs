//! Debug visualization export for the grasp preshaping pipeline.
//!
//! When `config::DEBUG_VISUALIZATION` is `true`, the pipeline writes a single
//! `.npz` file containing all intermediate results so they can be inspected
//! interactively in Python.

use crate::pointcloud_helper::{Aabb, Camera, PointCloud, Tsdf};
use nalgebra::Vector3;
use npyz::npz::NpzWriter;
use npyz::WriterBuilder;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::Path;

/// A single scored grasp candidate, ready for serialization.
/// Constructed by the caller from the private `ScoredGrasp` in `c_api.rs`.
#[derive(Debug, Clone)]
pub struct ScoredGraspExport {
    /// Index into the samples array (1:1 mapping since unified sampling).
    pub sample_index: usize,
    /// 1 = cylindrical, 2 = pinch, 3 = lateral.
    pub grasp_type_i32: i32,
    pub closure_amount: f64,
    pub alignment_score: f64,
    pub force_closure_score: f64,
    pub contact_count_score: f64,
    pub found_collision: bool,
    pub combined_score: f64,
    pub sample_probability: f64,
    /// Row-major 4x4 SE(3) matrix for the sample's hand base pose.
    pub pose_se3: [f64; 16],
    /// Wrist rotation angle around local Y axis in radians.
    pub wrist_rotation: f64,
}

/// Collected artifacts from one pipeline invocation.
pub struct DebugDump<'a> {
    pub tsdf: &'a Tsdf,
    pub point_cloud: &'a PointCloud,
    pub roi: &'a Aabb,
    pub cameras: &'a [Camera],
    pub scored_grasps: &'a [ScoredGraspExport],
    /// Input pose: [px, py, pz, qx, qy, qz, qw].
    pub input_pose: [f64; 7],
    /// Input twist: [lx, ly, lz, ax, ay, az].
    pub input_twist: [f64; 6],
}

/// Write all debug data to a single `.npz` file at `path`.
///
/// The file contains these named arrays:
///
/// | Name              | dtype | Shape       | Contents |
/// |-------------------|-------|-------------|----------|
/// | `tsdf_volume`     | f32   | (W*H*D,)    | Flat distance field |
/// | `tsdf_metadata`   | f64   | (7,)        | [origin_xyz, resolution, W, H, D] |
/// | `point_cloud`     | f32   | (N*3,)      | Interleaved xyz |
/// | `roi_aabb`        | f32   | (6,)        | [min_xyz, max_xyz] |
/// | `cameras`         | f32   | (C*3,)      | Camera positions |
/// | `input_pose`      | f64   | (7,)        | [px,py,pz,qx,qy,qz,qw] |
/// | `input_twist`     | f64   | (6,)        | [lx,ly,lz,ax,ay,az] |
/// | `scored_grasps`   | f64   | (M*26,)     | Flat rows (see below) |
///
/// Each scored_grasps row has 26 columns:
///   [sample_idx, grasp_type, closure, alignment, force_closure,
///    contact_count, found_collision(0|1), combined, probability, wrist_rotation,
///    pose_4x4(16)]
pub fn export_npz(dump: &DebugDump, path: &Path) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }

    let mut npz: NpzWriter<BufWriter<File>> = NpzWriter::create(path)?;

    // --- tsdf_volume (f32, flat) ---
    {
        let (w, h, d) = dump.tsdf.dimensions();
        let total = w * h * d;
        let mut writer = npz
            .array::<f32>("tsdf_volume", Default::default())?
            .default_dtype()
            .shape(&[total as u64])
            .begin_nd()?;
        for &v in dump.tsdf.data() {
            writer.push(&v)?;
        }
        writer.finish()?;
    }

    // --- tsdf_metadata (f64, 7) ---
    {
        let (w, h, d) = dump.tsdf.dimensions();
        let origin: Vector3<f32> = dump.tsdf.origin();
        let meta: [f64; 7] = [
            origin.x as f64,
            origin.y as f64,
            origin.z as f64,
            dump.tsdf.resolution() as f64,
            w as f64,
            h as f64,
            d as f64,
        ];
        let mut writer = npz
            .array::<f64>("tsdf_metadata", Default::default())?
            .default_dtype()
            .shape(&[7])
            .begin_nd()?;
        for &v in &meta {
            writer.push(&v)?;
        }
        writer.finish()?;
    }

    // --- point_cloud (f32, N*3 flat) ---
    {
        let n = dump.point_cloud.points.len();
        let mut writer = npz
            .array::<f32>("point_cloud", Default::default())?
            .default_dtype()
            .shape(&[(n * 3) as u64])
            .begin_nd()?;
        for p in &dump.point_cloud.points {
            writer.push(&p.x)?;
            writer.push(&p.y)?;
            writer.push(&p.z)?;
        }
        writer.finish()?;
    }

    // --- roi_aabb (f32, 6) ---
    {
        let aabb: [f32; 6] = [
            dump.roi.min.x,
            dump.roi.min.y,
            dump.roi.min.z,
            dump.roi.max.x,
            dump.roi.max.y,
            dump.roi.max.z,
        ];
        let mut writer = npz
            .array::<f32>("roi_aabb", Default::default())?
            .default_dtype()
            .shape(&[6])
            .begin_nd()?;
        for &v in &aabb {
            writer.push(&v)?;
        }
        writer.finish()?;
    }

    // --- cameras (f32, C*3 flat) ---
    {
        let total = dump.cameras.len() * 3;
        let mut writer = npz
            .array::<f32>("cameras", Default::default())?
            .default_dtype()
            .shape(&[total as u64])
            .begin_nd()?;
        for cam in dump.cameras {
            writer.push(&cam.position.x)?;
            writer.push(&cam.position.y)?;
            writer.push(&cam.position.z)?;
        }
        writer.finish()?;
    }

    // --- input_pose (f64, 7) ---
    {
        let mut writer = npz
            .array::<f64>("input_pose", Default::default())?
            .default_dtype()
            .shape(&[7])
            .begin_nd()?;
        for &v in &dump.input_pose {
            writer.push(&v)?;
        }
        writer.finish()?;
    }

    // --- input_twist (f64, 6) ---
    {
        let mut writer = npz
            .array::<f64>("input_twist", Default::default())?
            .default_dtype()
            .shape(&[6])
            .begin_nd()?;
        for &v in &dump.input_twist {
            writer.push(&v)?;
        }
        writer.finish()?;
    }

    // --- scored_grasps (f64, M*26 flat) ---
    // Columns per row (26 total):
    //   [0]  sample_index
    //   [1]  grasp_type (1=cyl, 2=pinch, 3=lat)
    //   [2]  closure_amount
    //   [3]  alignment_score
    //   [4]  force_closure_score
    //   [5]  contact_count_score
    //   [6]  found_collision (0.0 or 1.0)
    //   [7]  combined_score
    //   [8]  sample_probability
    //   [9]  wrist_rotation (radians)
    //   [10..26]  pose_se3 row-major 4x4
    {
        let m = dump.scored_grasps.len();
        let total = m * 26;
        let mut writer = npz
            .array::<f64>("scored_grasps", Default::default())?
            .default_dtype()
            .shape(&[total as u64])
            .begin_nd()?;
        for g in dump.scored_grasps {
            writer.push(&(g.sample_index as f64))?;
            writer.push(&(g.grasp_type_i32 as f64))?;
            writer.push(&g.closure_amount)?;
            writer.push(&g.alignment_score)?;
            writer.push(&g.force_closure_score)?;
            writer.push(&g.contact_count_score)?;
            writer.push(&if g.found_collision { 1.0f64 } else { 0.0f64 })?;
            writer.push(&g.combined_score)?;
            writer.push(&g.sample_probability)?;
            writer.push(&g.wrist_rotation)?;
            for &v in &g.pose_se3 {
                writer.push(&v)?;
            }
        }
        writer.finish()?;
    }

    // NpzWriter is dropped here, which finishes the zip archive.
    Ok(())
}

/// Generate a timestamped output path under the crate's `data/debug/` directory.
pub fn debug_output_path() -> std::path::PathBuf {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let base = std::path::Path::new(manifest_dir).join(crate::config::DEBUG_OUTPUT_DIR);

    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();

    let date_part = {
        // Simple UTC timestamp formatting without chrono.
        let days_since_epoch = secs / 86400;
        let time_of_day = secs % 86400;
        let hours = time_of_day / 3600;
        let minutes = (time_of_day % 3600) / 60;
        let seconds = time_of_day % 60;

        // Compute year/month/day from days since epoch (simplified Gregorian).
        let (year, month, day) = days_to_ymd(days_since_epoch);
        format!(
            "{}{:02}{:02}_{:02}{:02}{:02}",
            year, month, day, hours, minutes, seconds
        )
    };

    base.join(format!("grasp_dump_{}.npz", date_part))
}

/// Convert days since Unix epoch to (year, month, day).
/// Simplified Gregorian calendar calculation.
fn days_to_ymd(mut days: u64) -> (u64, u64, u64) {
    // 400-year cycle has 146097 days
    let cycles = days / 146097;
    days %= 146097;

    // 100-year sub-cycle
    let mut year = cycles * 400;
    let century = days / 36524;
    let century = century.min(3);
    year += century * 100;
    days -= century * 36524;

    // 4-year sub-cycle
    let quad = days / 1461;
    year += quad * 4;
    days -= quad * 1461;

    // Single years
    let y = (days / 365).min(3);
    year += y;
    days -= y * 365;

    let leap = is_leap(year);
    let month_days = if leap {
        [31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    } else {
        [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    };

    let mut month = 1u64;
    let mut remaining = days;
    for &md in &month_days {
        if remaining < md {
            break;
        }
        remaining -= md;
        month += 1;
    }

    (year, month, remaining + 1)
}

fn is_leap(year: u64) -> bool {
    (year % 4 == 0 && year % 100 != 0) || year % 400 == 0
}
