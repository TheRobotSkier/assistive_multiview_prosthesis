use std::process::Command;

pub fn publish_float64_multi_array_once(topic: &str, value: f64) -> Result<(), String> {
    let payload = format!("{{data: [{:.6}]}}", value);
    let status = Command::new("ros2")
        .arg("topic")
        .arg("pub")
        .arg("--once")
        .arg(topic)
        .arg("std_msgs/msg/Float64MultiArray")
        .arg(payload)
        .status()
        .map_err(|e| format!("Failed to spawn ros2 for {}: {}", topic, e))?;

    if status.success() {
        Ok(())
    } else {
        Err(format!(
            "ros2 topic pub failed for {} with exit code {:?}",
            topic,
            status.code()
        ))
    }
}
