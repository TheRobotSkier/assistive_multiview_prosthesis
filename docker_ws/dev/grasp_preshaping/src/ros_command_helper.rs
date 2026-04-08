use std::process::Command;

pub fn publish_float64_multi_array_once(topic: &str, value: f64) -> Result<(), String> {
    ensure_topic_has_subscriber(topic)?;

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

pub fn publish_joint_trajectory_once(
    topic: &str,
    joint_name: &str,
    position: f64,
    time_from_start_sec: f64,
) -> Result<(), String> {
    ensure_topic_has_subscriber(topic)?;

    let (sec, nanosec) = duration_parts(time_from_start_sec)?;
    let payload = format!(
        "{{joint_names: ['{}'], points: [{{positions: [{:.6}], time_from_start: {{sec: {}, nanosec: {}}}}}]}}",
        joint_name, position, sec, nanosec
    );

    let status = Command::new("ros2")
        .arg("topic")
        .arg("pub")
        .arg("--once")
        .arg(topic)
        .arg("trajectory_msgs/msg/JointTrajectory")
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

pub fn ensure_topic_has_subscriber(topic: &str) -> Result<(), String> {
    let sub_count = get_subscription_count(topic)?;
    if sub_count == 0 {
        return Err(format!(
            "Refusing to publish on '{}' because it currently has 0 subscribers. \
Expected an active controller subscriber. Start the correct launch stack for this backend, then retry.",
            topic
        ));
    }

    Ok(())
}

fn get_subscription_count(topic: &str) -> Result<u32, String> {
    let output = Command::new("ros2")
        .arg("topic")
        .arg("info")
        .arg(topic)
        .output()
        .map_err(|e| format!("Failed to run 'ros2 topic info {}': {}", topic, e))?;

    let stdout =
        String::from_utf8(output.stdout).map_err(|e| format!("Invalid UTF-8 in ros2 stdout: {}", e))?;
    let stderr =
        String::from_utf8(output.stderr).map_err(|e| format!("Invalid UTF-8 in ros2 stderr: {}", e))?;

    if !output.status.success() {
        return Err(format!(
            "ros2 topic info failed for '{}', code {:?}. stdout='{}' stderr='{}'",
            topic,
            output.status.code(),
            stdout.trim(),
            stderr.trim()
        ));
    }

    for line in stdout.lines() {
        let trimmed = line.trim();
        if let Some(rest) = trimmed.strip_prefix("Subscription count:") {
            let count = rest.trim().parse::<u32>().map_err(|e| {
                format!(
                    "Failed to parse subscription count for topic '{}': {} (line='{}')",
                    topic, e, trimmed
                )
            })?;
            return Ok(count);
        }
    }

    Err(format!(
        "Could not find 'Subscription count:' in 'ros2 topic info {}' output: {}",
        topic,
        stdout.trim()
    ))
}

fn duration_parts(seconds: f64) -> Result<(u32, u32), String> {
    if !seconds.is_finite() || seconds <= 0.0 {
        return Err(format!(
            "time_from_start_sec must be finite and > 0, got {}",
            seconds
        ));
    }

    let sec = seconds.floor();
    let mut nanosec = ((seconds - sec) * 1_000_000_000.0).round() as i64;
    let mut sec_u64 = sec as u64;

    if nanosec >= 1_000_000_000 {
        nanosec -= 1_000_000_000;
        sec_u64 = sec_u64.saturating_add(1);
    }

    if sec_u64 > u32::MAX as u64 {
        return Err(format!("time_from_start_sec is too large: {}", seconds));
    }

    Ok((sec_u64 as u32, nanosec as u32))
}
