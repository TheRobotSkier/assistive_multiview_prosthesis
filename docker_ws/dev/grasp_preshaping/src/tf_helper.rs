use nalgebra::{Matrix4, Quaternion, UnitQuaternion};
use serde::Deserialize;
use std::process::Command;

#[derive(Debug, Deserialize)]
struct TfMessage {
	transforms: Vec<TransformStamped>,
}

#[derive(Debug, Deserialize)]
struct TransformStamped {
	header: Header,
	child_frame_id: String,
	transform: Transform,
}

#[derive(Debug, Deserialize)]
struct Header {
	frame_id: String,
}

#[derive(Debug, Deserialize)]
struct Transform {
	translation: Vector3,
	rotation: QuaternionMsg,
}

#[derive(Debug, Deserialize)]
struct Vector3 {
	x: f64,
	y: f64,
	z: f64,
}

#[derive(Debug, Deserialize)]
struct QuaternionMsg {
	x: f64,
	y: f64,
	z: f64,
	w: f64,
}

fn normalize_frame_id(frame_id: &str) -> &str {
	frame_id.trim().trim_start_matches('/')
}

pub fn lookup_transform_matrix(topic: &str, parent_frame: &str, child_frame: &str) -> Result<Matrix4<f64>, String> {
	let filter = format!(
		"any(t.child_frame_id == \"{child}\" and t.header.frame_id == \"{parent}\" for t in m.transforms)",
		child = child_frame,
		parent = parent_frame,
	);

	let output = Command::new("ros2")
		.arg("topic")
		.arg("echo")
		.arg("--full-length")
		.arg("--once")
		.arg("--filter")
		.arg(filter)
		.arg(topic)
		.arg("tf2_msgs/msg/TFMessage")
		.output()
		.map_err(|e| format!("Failed to run ros2 topic echo for TF lookup: {}", e))?;

	if !output.status.success() {
		return Err(format!(
			"ros2 topic echo failed on topic '{}' with code {:?}",
			topic,
			output.status.code()
		));
	}

	let raw = String::from_utf8(output.stdout)
		.map_err(|e| format!("TF output is not valid UTF-8: {}", e))?;
	let normalized = raw
		.lines()
		.skip_while(|line| {
			let trimmed = line.trim();
			trimmed.is_empty() || trimmed == "---" || trimmed == "..."
		})
		.take_while(|line| {
			let trimmed = line.trim();
			trimmed != "---" && trimmed != "..."
		})
		.collect::<Vec<_>>()
		.join("\n");

	let tf_message: TfMessage = serde_yaml::from_str(&normalized)
		.map_err(|e| format!("Failed to parse TF YAML from '{}': {}\nRaw output:\n{}", topic, e, normalized))?;

	let transform = tf_message
		.transforms
		.iter()
		.find(|transform| {
			normalize_frame_id(&transform.header.frame_id) == normalize_frame_id(parent_frame)
				&& normalize_frame_id(&transform.child_frame_id) == normalize_frame_id(child_frame)
		})
		.ok_or_else(|| {
			format!(
				"No transform found on '{}' from '{}' to '{}'",
				topic, parent_frame, child_frame
			)
		})?;

	let rotation = UnitQuaternion::new_normalize(Quaternion::new(
		transform.transform.rotation.w,
		transform.transform.rotation.x,
		transform.transform.rotation.y,
		transform.transform.rotation.z,
	));

	let mut matrix = rotation.to_homogeneous();
	matrix[(0, 3)] = transform.transform.translation.x;
	matrix[(1, 3)] = transform.transform.translation.y;
	matrix[(2, 3)] = transform.transform.translation.z;
	Ok(matrix)
}
