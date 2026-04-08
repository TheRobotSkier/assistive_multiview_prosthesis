#!/usr/bin/env python3
"""Publish simulated depth outputs and an object-only world-frame point cloud."""

from __future__ import annotations

import re
import threading
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField


def absolutize_file_refs(xml_text: str, base_dir: Path) -> str:
    def repl(match: re.Match[str]) -> str:
        rel = match.group(1)
        p = Path(rel)
        if p.is_absolute() or "://" in rel:
            return match.group(0)
        abs_path = (base_dir / p).resolve()
        return f'file="{abs_path.as_posix()}"'

    return re.sub(r'file="([^"]+)"', repl, xml_text)


def absolutize_meshdir(xml_text: str, base_dir: Path) -> str:
    def repl(match: re.Match[str]) -> str:
        meshdir = match.group(1)
        p = Path(meshdir)
        if p.is_absolute() or "://" in meshdir:
            return match.group(0)
        abs_path = (base_dir / p).resolve()
        return f'meshdir="{abs_path.as_posix()}"'

    return re.sub(r'meshdir="([^"]+)"', repl, xml_text)


def build_no_plugin_scene(scene_xml: Path, out_dir: Path) -> Path:
    scene_text = scene_xml.read_text(encoding="utf-8")
    include_match = re.search(r'<include\s+file="([^"]+)"\s*/>', scene_text)
    if include_match is None:
        raise ValueError(f"Could not find include file in scene: {scene_xml}")

    include_file = include_match.group(1)
    hand_xml = (scene_xml.parent / include_file).resolve()
    if not hand_xml.exists():
        raise FileNotFoundError(f"Hand XML not found: {hand_xml}")

    hand_text = hand_xml.read_text(encoding="utf-8")
    hand_text = re.sub(r"<extension>.*?</extension>\s*", "", hand_text, flags=re.DOTALL)
    side_suffix = "_l" if 'j_index_fle_l' in hand_text else "_r"
    hand_text = re.sub(
        r'<plugin[^>]*name="index_fle_pos_[rl]"[^>]*/>',
        f'<position name="index_fle_pos{side_suffix}" joint="j_index_fle{side_suffix}" inheritrange="1"/>',
        hand_text,
    )
    hand_text = absolutize_meshdir(hand_text, hand_xml.parent)

    generated_dir = (out_dir / "_generated_model").resolve()
    generated_dir.mkdir(parents=True, exist_ok=True)

    generated_hand = generated_dir / "mia_hand_no_plugin.xml"
    generated_hand.write_text(hand_text, encoding="utf-8")

    generated_scene_text = re.sub(
        r'<include\s+file="[^"]+"\s*/>',
        f'<include file="{generated_hand.as_posix()}"/>',
        scene_text,
        count=1,
    )
    generated_scene_text = absolutize_file_refs(generated_scene_text, scene_xml.parent)
    generated_scene = generated_dir / f"{scene_xml.stem}_no_plugin.xml"
    generated_scene.write_text(generated_scene_text, encoding="utf-8")
    return generated_scene


def load_model_with_fallback(scene_xml: Path, out_dir: Path) -> tuple[mujoco.MjModel, Path, bool]:
    xml_path = scene_xml.resolve()
    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        return model, xml_path, False
    except ValueError as exc:
        if "index_thumb_actuator" not in str(exc):
            raise
        generated_path = build_no_plugin_scene(xml_path, out_dir)
        model = mujoco.MjModel.from_xml_path(str(generated_path))
        return model, generated_path, True


def derive_thumb_opposition(index_angle: float, min_angle: float, max_angle: float) -> float:
    start_index_angle = -0.06
    end_index_angle = 0.08
    if end_index_angle <= start_index_angle:
        return min_angle

    alpha = (index_angle - start_index_angle) / (end_index_angle - start_index_angle)
    alpha = max(0.0, min(1.0, alpha))
    return min_angle + alpha * (max_angle - min_angle)


class MujocoDepthPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_depth_publisher")

        self.declare_parameter("xml_model_path", "")
        self.declare_parameter("output_dir", "/tmp/mia_hand_mujoco_depth")
        self.declare_parameter("camera_name", "front_depth_cam")
        self.declare_parameter("camera_frame_id", "mujoco_front_depth_cam")
        self.declare_parameter("world_frame_id", "world")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("depth_image_topic", "/mujoco/depth/image")
        self.declare_parameter("depth_camera_info_topic", "/mujoco/depth/camera_info")
        self.declare_parameter("depth_camera_pointcloud_topic", "/mujoco/depth/points_camera")
        self.declare_parameter("segmented_pointcloud_topic", "/segmented_object_cloud")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("max_depth", 1.5)
        self.declare_parameter("pointcloud_stride", 2)
        self.declare_parameter("warmup_steps", 0)
        self.declare_parameter("laterality", "right")
        self.declare_parameter("prefix", "")
        self.declare_parameter("target_geom_name", "target_sphere")
        self.declare_parameter("publish_depth_image", True)
        self.declare_parameter("publish_camera_info", True)
        self.declare_parameter("publish_camera_cloud", True)
        self.declare_parameter("wait_for_joint_state", True)

        xml_model_path_value = str(self.get_parameter("xml_model_path").value).strip()
        if not xml_model_path_value:
            raise ValueError("xml_model_path parameter must not be empty")
        xml_model_path = Path(xml_model_path_value)
        output_dir = Path(self.get_parameter("output_dir").value)
        output_dir.mkdir(parents=True, exist_ok=True)

        self.model, used_xml, fallback_used = load_model_with_fallback(xml_model_path, output_dir)
        self.data = mujoco.MjData(self.model)

        self.camera_name = str(self.get_parameter("camera_name").value)
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.world_frame_id = str(self.get_parameter("world_frame_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.pointcloud_stride = max(1, int(self.get_parameter("pointcloud_stride").value))
        self.publish_depth_image = bool(self.get_parameter("publish_depth_image").value)
        self.publish_camera_info = bool(self.get_parameter("publish_camera_info").value)
        self.publish_camera_cloud = bool(self.get_parameter("publish_camera_cloud").value)
        self.wait_for_joint_state = bool(self.get_parameter("wait_for_joint_state").value)
        self.target_geom_name = str(self.get_parameter("target_geom_name").value)
        self.prefix = str(self.get_parameter("prefix").value)
        self.laterality = str(self.get_parameter("laterality").value)

        self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        self.renderer.enable_depth_rendering()

        self.cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)
        if self.cam_id < 0:
            raise ValueError(f"Camera '{self.camera_name}' not found in {used_xml}")

        self.target_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, self.target_geom_name)
        if self.target_geom_id < 0:
            raise ValueError(f"Target geom '{self.target_geom_name}' not found in {used_xml}")

        self.fovy = float(self.model.cam_fovy[self.cam_id])
        self.fx = 0.5 * self.width / np.tan(np.deg2rad(self.fovy) * 0.5)
        self.fy = self.fx
        self.cx = (self.width - 1) * 0.5
        self.cy = (self.height - 1) * 0.5
        self.geomgroup = np.ones(6, dtype=np.uint8)

        self.lock = threading.Lock()
        self.latest_joint_positions: dict[str, float] = {}
        self.have_joint_state = False
        self.warned_waiting_for_joint_state = False

        self.ros_to_mj = self._build_joint_name_map()
        self.thumb_opp_qposadr = self._qposadr_for_joint(self.ros_to_mj.get(self.prefix + "j_thumb_opp", ""))
        self.thumb_opp_range = self._joint_range(self.ros_to_mj.get(self.prefix + "j_thumb_opp", ""))

        for _ in range(max(0, int(self.get_parameter("warmup_steps").value))):
            mujoco.mj_forward(self.model, self.data)

        self.depth_pub = self.create_publisher(Image, str(self.get_parameter("depth_image_topic").value), 10)
        self.info_pub = self.create_publisher(CameraInfo, str(self.get_parameter("depth_camera_info_topic").value), 10)
        self.camera_cloud_pub = self.create_publisher(PointCloud2, str(self.get_parameter("depth_camera_pointcloud_topic").value), 10)
        self.world_cloud_pub = self.create_publisher(PointCloud2, str(self.get_parameter("segmented_pointcloud_topic").value), 10)

        self.joint_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self.on_joint_state,
            10,
        )

        timer_period = 1.0 / max(1e-3, self.publish_hz)
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().info(f"Loaded shadow MuJoCo model from: {used_xml}")
        if fallback_used:
            self.get_logger().warn("Using generated no-plugin MuJoCo model fallback for depth publishing.")

    def destroy_node(self) -> bool:
        self.renderer.close()
        return super().destroy_node()

    def _build_joint_name_map(self) -> dict[str, str]:
        suffix = "_r" if self.laterality == "right" else "_l"
        mapping = {}
        for base_name in ["j_thumb_fle", "j_index_fle", "j_mrl_fle", "j_thumb_opp"]:
            mapping[self.prefix + base_name] = base_name + suffix
        return mapping

    def _qposadr_for_joint(self, joint_name: str) -> int | None:
        if not joint_name:
            return None
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            return None
        return int(self.model.jnt_qposadr[joint_id])

    def _joint_range(self, joint_name: str) -> tuple[float, float] | None:
        if not joint_name:
            return None
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            return None
        return (
            float(self.model.jnt_range[joint_id][0]),
            float(self.model.jnt_range[joint_id][1]),
        )

    def on_joint_state(self, msg: JointState) -> None:
        with self.lock:
            self.latest_joint_positions = {
                name: position
                for name, position in zip(msg.name, msg.position)
            }
            self.have_joint_state = True

    def on_timer(self) -> None:
        with self.lock:
            joint_positions = dict(self.latest_joint_positions)
            have_joint_state = self.have_joint_state

        if self.wait_for_joint_state and not have_joint_state:
            if not self.warned_waiting_for_joint_state:
                self.get_logger().info("Waiting for /joint_states before publishing depth cloud.")
                self.warned_waiting_for_joint_state = True
            return

        self._apply_joint_positions(joint_positions)
        mujoco.mj_forward(self.model, self.data)

        self.renderer.update_scene(self.data, camera=self.camera_name)
        depth = self.renderer.render().astype(np.float32)
        invalid = (depth <= 0.0) | (~np.isfinite(depth)) | (depth > self.max_depth)
        depth[invalid] = 0.0

        stamp = self.get_clock().now().to_msg()

        if self.publish_depth_image:
            self.depth_pub.publish(self.make_depth_image(depth, stamp))
        if self.publish_camera_info:
            self.info_pub.publish(self.make_camera_info(stamp))

        points_cam_ros = self.depth_to_camera_points(depth)
        points_cam_ros = self.filter_points_by_target_geom(points_cam_ros)
        points_world = self.camera_points_to_world(points_cam_ros)

        if self.publish_camera_cloud:
            self.camera_cloud_pub.publish(self.make_pointcloud2(points_cam_ros, stamp, self.camera_frame_id))
        self.world_cloud_pub.publish(self.make_pointcloud2(points_world, stamp, self.world_frame_id))

    def _apply_joint_positions(self, joint_positions: dict[str, float]) -> None:
        for ros_name, mj_name in self.ros_to_mj.items():
            if ros_name not in joint_positions or ros_name.endswith("j_thumb_opp"):
                continue
            qposadr = self._qposadr_for_joint(mj_name)
            if qposadr is None:
                continue
            self.data.qpos[qposadr] = float(joint_positions[ros_name])

        thumb_opp_ros_name = self.prefix + "j_thumb_opp"
        thumb_opp_value = joint_positions.get(thumb_opp_ros_name)
        if thumb_opp_value is None and self.thumb_opp_range is not None:
            index_value = joint_positions.get(self.prefix + "j_index_fle")
            if index_value is not None:
                thumb_opp_value = derive_thumb_opposition(index_value, *self.thumb_opp_range)

        if thumb_opp_value is not None and self.thumb_opp_qposadr is not None:
            self.data.qpos[self.thumb_opp_qposadr] = float(thumb_opp_value)

    def depth_to_camera_points(self, depth: np.ndarray) -> np.ndarray:
        v, u = np.indices((self.height, self.width))
        valid = depth > 0.0

        z = depth[valid]
        x = (u[valid] - self.cx) * z / self.fx
        y = -(v[valid] - self.cy) * z / self.fy
        points = np.column_stack((x, y, z)).astype(np.float32)

        if self.pointcloud_stride > 1 and points.shape[0] > 0:
            points = points[:: self.pointcloud_stride]
        return points

    def filter_points_by_target_geom(self, points_cam_ros: np.ndarray) -> np.ndarray:
        if points_cam_ros.shape[0] == 0:
            return points_cam_ros

        cam_pos = self.data.cam_xpos[self.cam_id].copy()
        cam_rot = self.data.cam_xmat[self.cam_id].reshape(3, 3).copy()

        keep = np.zeros(points_cam_ros.shape[0], dtype=bool)
        geomid = np.array([-1], dtype=np.int32)

        for i, point in enumerate(points_cam_ros):
            ray_cam = np.array([float(point[0]), float(point[1]), -float(point[2])], dtype=np.float64)
            norm = np.linalg.norm(ray_cam)
            if norm < 1e-12:
                continue
            ray_cam /= norm

            ray_world = cam_rot @ ray_cam
            dist = mujoco.mj_ray(
                self.model,
                self.data,
                cam_pos,
                ray_world,
                self.geomgroup,
                1,
                -1,
                geomid,
            )

            if dist < 0 or geomid[0] < 0:
                continue
            keep[i] = int(geomid[0]) == self.target_geom_id

        return points_cam_ros[keep]

    def camera_points_to_world(self, points_cam_ros: np.ndarray) -> np.ndarray:
        if points_cam_ros.shape[0] == 0:
            return points_cam_ros

        points_cam_mj = points_cam_ros.copy()
        points_cam_mj[:, 2] *= -1.0
        rot = self.data.cam_xmat[self.cam_id].reshape(3, 3).copy()
        cam_pos = self.data.cam_xpos[self.cam_id].copy()
        return (points_cam_mj @ rot.T + cam_pos).astype(np.float32)

    def make_depth_image(self, depth: np.ndarray, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = "32FC1"
        msg.is_bigendian = False
        msg.step = self.width * 4
        msg.data = np.ascontiguousarray(depth).tobytes()
        return msg

    def make_camera_info(self, stamp) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame_id
        msg.height = self.height
        msg.width = self.width
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [
            float(self.fx), 0.0, float(self.cx),
            0.0, float(self.fy), float(self.cy),
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            float(self.fx), 0.0, float(self.cx), 0.0,
            0.0, float(self.fy), float(self.cy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def make_pointcloud2(self, points_xyz: np.ndarray, stamp, frame_id: str) -> PointCloud2:
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = int(points_xyz.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.ascontiguousarray(points_xyz, dtype=np.float32).tobytes()
        return msg


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = MujocoDepthPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())