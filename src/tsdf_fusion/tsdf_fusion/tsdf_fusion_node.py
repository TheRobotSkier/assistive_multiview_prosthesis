#!/usr/bin/env python3
"""tsdf_fusion_node — ROS 2 wrapper for on-demand TSDF fusion.

On a ``TriggerGraspFusion`` request the node:
1. Calls the keyframe buffer service ``GetKeyframesInROI`` for keyframes near
   the hit point.
2. Deserialises the returned ``sensor_fusion_msgs/Keyframe`` messages into
   :class:`keyframe_buffer.keyframe.Keyframe` objects.
3. Calls :func:`tsdf_fusion.tsdf_fusion_core.fuse_object_cloud` with an HTTP
   MobileSAM client as the segmentation function.
4. Converts the result to ``sensor_msgs/PointCloud2``.
5. Publishes to ``/segmentation/object_cloud`` AND returns it in the response.

The pure-logic core has zero ROS imports; this node is a thin plumbing layer.

Reference: V6 plan §6.5, §6.9.
"""

from __future__ import annotations

import base64
import time
from typing import List, Optional

import numpy as np

# Pure-logic helpers (ROS-free).
from keyframe_buffer.keyframe import Keyframe
from tsdf_fusion.tsdf_fusion_core import (
    fuse_object_cloud,
    fuse_scene_preview,
    FuseResult,
)


__all__ = ["create_node", "main"]


# ---------------------------------------------------------------------------
# MobileSAM HTTP client
# ---------------------------------------------------------------------------

class SamHttpClient:
    """HTTP client wrapping the MobileSAM ``/segment_2d`` endpoint.

    Exposes a ``sam_segment_fn(image, uv, dilation_px) -> mask`` callable that
    :func:`fuse_object_cloud` expects.
    """

    def __init__(self, base_url: str, timeout_s: float = 5.0,
                 logger=None):
        import requests  # lazy import

        self._base_url = base_url.rstrip("/")
        self._timeout_s = float(timeout_s)
        self._requests = requests
        self._log = logger

    def health(self) -> bool:
        """Ping ``/health``.  Returns ``True`` if the server responds."""
        try:
            resp = self._requests.get(
                f"{self._base_url}/health", timeout=self._timeout_s)
            return resp.status_code == 200
        except Exception:
            return False

    def __call__(self, image: np.ndarray, uv, dilation_px: int = 15
                 ) -> np.ndarray:
        """Call MobileSAM ``/segment_2d`` and return a ``(H, W)`` bool mask.

        Parameters
        ----------
        image : ndarray (H, W, 3) uint8
            RGB image.
        uv : (int, int)
            ``(u, v)`` click pixel.
        dilation_px : int
            Mask dilation in pixels.

        Returns
        -------
        mask : ndarray (H, W) bool
        """
        H, W = int(image.shape[0]), int(image.shape[1])
        u, v = int(uv[0]), int(uv[1])

        # Encode the image as base64 JPEG.
        try:
            import cv2  # lazy import

            ok, buf = cv2.imencode(".jpg", image[:, :, ::-1])  # RGB→BGR for cv2
            if not ok:
                return np.zeros((H, W), dtype=bool)
            img_b64 = base64.b64encode(buf.tobytes()).decode("ascii")
        except Exception:
            # Fallback: raw bytes (less efficient).
            img_b64 = base64.b64encode(
                np.ascontiguousarray(image.astype(np.uint8)).tobytes()
            ).decode("ascii")

        payload = {
            "image": img_b64,
            "point": [u, v],
            "dilation_px": int(dilation_px),
            "width": W,
            "height": H,
        }

        try:
            resp = self._requests.post(
                f"{self._base_url}/segment_2d",
                json=payload, timeout=self._timeout_s)
            if resp.status_code != 200:
                return np.zeros((H, W), dtype=bool)
            data = resp.json()
        except Exception as exc:
            if self._log is not None:
                self._log.warn(f"SAM /segment_2d failed: {exc}")
            return np.zeros((H, W), dtype=bool)

        # Expected response: {"mask": [[0,1,...], ...]} or
        # {"mask": "<base64 uint8>", "width": W, "height": H}.
        mask_raw = data.get("mask")
        if mask_raw is None:
            return np.zeros((H, W), dtype=bool)

        if isinstance(mask_raw, str):
            # base64-encoded flat byte array.
            flat = np.frombuffer(base64.b64decode(mask_raw), dtype=np.uint8)
            mask = flat.reshape(H, W).astype(bool)
        elif isinstance(mask_raw, list):
            mask = np.asarray(mask_raw, dtype=np.uint8).reshape(H, W).astype(bool)
        else:
            return np.zeros((H, W), dtype=bool)

        return mask


# ---------------------------------------------------------------------------
# Keyframe message deserialisation (reverse of keyframe_buffer_node's serialiser)
# ---------------------------------------------------------------------------

def _keyframe_from_msg(msg) -> Keyframe:
    """Deserialise a ``sensor_fusion_msgs/Keyframe`` into a ``Keyframe``."""
    # --- Cloud ---
    pc2 = msg.cloud
    n = pc2.width * pc2.height
    step = pc2.point_step
    fields = {f.name: f for f in pc2.fields}

    def _col_f32(name: str) -> np.ndarray:
        off = fields[name].offset
        if n == 0:
            return np.zeros((0,), dtype=np.float32)
        raw = np.frombuffer(bytes(pc2.data), dtype=np.uint8).reshape(n, step)
        return np.frombuffer(
            raw[:, off:off + 4].copy().tobytes(), dtype=np.float32)

    if n > 0:
        xyz = np.column_stack([_col_f32("x"), _col_f32("y"), _col_f32("z")])
        rgb = np.zeros((n, 3), dtype=np.uint8)
        if "rgb" in fields:
            packed = _col_f32("rgb").view(np.uint32)
            rgb[:, 0] = (packed >> 16) & 0xFF
            rgb[:, 1] = (packed >> 8) & 0xFF
            rgb[:, 2] = packed & 0xFF
    else:
        xyz = np.zeros((0, 3), dtype=np.float32)
        rgb = np.zeros((0, 3), dtype=np.uint8)

    # --- Image ---
    img_msg = msg.image
    raw = np.frombuffer(bytes(img_msg.data), dtype=np.uint8)
    enc = img_msg.encoding
    if enc in ("rgb8", "bgr8"):
        image = raw.reshape(img_msg.height, img_msg.width, 3)
        if enc == "bgr8":
            image = image[:, :, ::-1]
    elif enc in ("rgba8", "bgra8"):
        image = raw.reshape(img_msg.height, img_msg.width, 4)[:, :, :3]
        if enc == "bgra8":
            image = image[:, :, ::-1]
    else:
        image = raw.reshape(img_msg.height, img_msg.width, 3)

    # --- Intrinsics ---
    K = np.array(msg.camera_info.k, dtype=np.float64).reshape(3, 3)

    # --- Pose ---
    p = msg.pose.position
    q = msg.pose.orientation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    nrm = qx * qx + qy * qy + qz * qz + qw * qw
    pose = np.eye(4, dtype=np.float64)
    if nrm >= 1e-15:
        s = 2.0 / nrm
        xs, ys, zs = qx * s, qy * s, qz * s
        wx, wy, wz = qw * xs, qw * ys, qw * zs
        xx, xy, xz = qx * xs, qx * ys, qx * zs
        yy, yz, zz = qy * ys, qy * zs, qz * zs
        pose[:3, :3] = np.array([
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ])
    pose[:3, 3] = [p.x, p.y, p.z]

    # --- Timestamp ---
    ts = float(msg.timestamp.sec) + float(msg.timestamp.nanosec) * 1e-9

    return Keyframe(
        timestamp=ts,
        camera_id=str(msg.camera_id),
        cloud_xyz=xyz,
        cloud_rgb=rgb,
        image=image.copy(),
        K=K,
        pose=pose,
        organized=bool(msg.organized),
    )


# ---------------------------------------------------------------------------
# PointCloud2 builder (xyz + rgb)
# ---------------------------------------------------------------------------

def _build_xyzrgb_cloud(xyz: np.ndarray, rgb: np.ndarray, header) -> object:
    """Build an XYZRGB ``sensor_msgs/PointCloud2`` from numpy arrays."""
    from sensor_msgs.msg import PointCloud2, PointField

    n = int(xyz.shape[0])
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True

    if n == 0:
        msg.data = b""
        return msg

    xyz32 = np.ascontiguousarray(xyz, dtype=np.float32)
    rgb_u32 = np.ascontiguousarray(rgb, dtype=np.uint32)
    rgb_packed = (
        rgb_u32[:, 0] * np.uint32(1 << 16)
        + rgb_u32[:, 1] * np.uint32(1 << 8)
        + rgb_u32[:, 2]
    ).astype(np.uint32)
    buf = np.zeros((n, 4), dtype=np.float32)
    buf[:, :3] = xyz32
    buf[:, 3] = rgb_packed.view(np.float32)
    msg.data = np.ascontiguousarray(buf).tobytes()
    return msg


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

def _import_ros():
    """Import ROS 2 modules lazily (host-testable core without ROS)."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import Point
    from std_msgs.msg import Header
    return rclpy, Node, PointCloud2, Point, Header


DEFAULT_PARAMS = {
    "keyframe_service": "/keyframe_buffer/get_in_roi",
    "sam_inference_url": "http://127.0.0.1:5679",
    "sam_timeout_s": 5.0,
    "roi_radius_m": 0.15,
    "hit_point_shift_m": 0.015,
    "mask_dilation_px": 15,
    "voxel_size_m": 0.005,
    "sdf_trunc_m": 0.02,
    "dbscan_eps_m": 0.02,
    "dbscan_min_points": 10,
    "output_topic": "/segmentation/object_cloud",
    "world_frame": "marker_map",
    # ── Depth densification for sparse clouds ───────────────────────────
    # Sparse keyframe clouds cover only 10-15 % of the image plane after
    # decimation.  Splatting + hole-filling close intra-surface gaps so the
    # Open3D TSDF ray-caster produces actual geometry instead of "NO GEOMETRY".
    "depth_splat_radius_px": 2,
    "depth_hole_fill_px": 5.0,
    # ── Preview mode: timer-driven scene-level TSDF fusion ─────────────
    # When preview_mode is True the node runs a lightweight 1 Hz loop that
    # integrates ALL keyframes (no hit point, no SAM) and publishes a fused
    # scene cloud.  See plans/2026-06-16-tsdf-scene-preview-fusion-1.0.md.
    "preview_mode": False,
    "preview_rate_hz": 1.0,
    "preview_output_topic": "/fused_pointcloud",
    "preview_keyframe_service": "/keyframe_buffer/get_all",
    "preview_workspace_bbox_min": [-0.5, -0.5, -0.2],
    "preview_workspace_bbox_max": [0.8, 0.5, 0.6],
    "preview_enable_bbox_crop": True,
    "preview_enable_dbscan_cleanup": True,
}


def create_node():
    """Build and return the ``TsdfFusionNode`` (ROS 2 Node subclass)."""
    (rclpy, Node, PointCloud2, Point, Header) = _import_ros()

    try:
        from sensor_fusion_msgs.srv import (
            GetKeyframesInROI, GetAllKeyframes, TriggerGraspFusion)
    except ImportError:
        GetKeyframesInROI = None  # type: ignore
        GetAllKeyframes = None  # type: ignore
        TriggerGraspFusion = None  # type: ignore

    class TsdfFusionNode(Node):
        """ROS 2 node wrapping :func:`fuse_object_cloud`."""

        def __init__(self):
            super().__init__("tsdf_fusion")

            # Reentrant callback group — required for service calls from
            # inside timer callbacks (e.g. GetAllKeyframes in _preview_tick).
            from rclpy.callback_groups import ReentrantCallbackGroup

            self._reentrant_group = ReentrantCallbackGroup()

            # ── Declare parameters (V6 §6.9) ────────────────────────────
            for key, default in DEFAULT_PARAMS.items():
                self.declare_parameter(key, default)

            p = lambda k: self.get_parameter(k).value  # noqa: E731

            self._roi_radius = float(p("roi_radius_m"))
            self._hit_shift = float(p("hit_point_shift_m"))
            self._mask_dilation = int(p("mask_dilation_px"))
            self._voxel_size = float(p("voxel_size_m"))
            self._sdf_trunc = float(p("sdf_trunc_m"))
            self._dbscan_eps = float(p("dbscan_eps_m"))
            self._dbscan_min = int(p("dbscan_min_points"))
            self._world_frame = str(p("world_frame"))
            self._depth_splat = int(p("depth_splat_radius_px"))
            self._depth_hole_fill = float(p("depth_hole_fill_px"))

            # ── Branch: preview mode vs. grasp mode ──────────────────────
            self._preview_mode = bool(p("preview_mode"))

            if self._preview_mode:
                self._init_preview_mode(p)
            else:
                self._init_grasp_mode(p, TriggerGraspFusion, GetKeyframesInROI)

        # ── Mode initialisers ──────────────────────────────────────────

        def _init_grasp_mode(self, p, TriggerGraspFusion, GetKeyframesInROI):
            """Initialise the on-demand grasp-fusion path (legacy V6)."""
            # ── SAM client ──────────────────────────────────────────────
            self._sam = SamHttpClient(
                base_url=str(p("sam_inference_url")),
                timeout_s=float(p("sam_timeout_s")),
                logger=self.get_logger(),
            )

            # ── Publisher ───────────────────────────────────────────────
            self._pub = self.create_publisher(
                PointCloud2, str(p("output_topic")), 10)

            # ── Service ─────────────────────────────────────────────────
            if TriggerGraspFusion is not None:
                self._srv = self.create_service(
                    TriggerGraspFusion,
                    "~/trigger",
                    self._handle_trigger,
                )
                self.get_logger().info(
                    "Service advertised: /tsdf_fusion/trigger")
            else:
                self._srv = None
                self.get_logger().warn(
                    "sensor_fusion_msgs not available — TriggerGraspFusion "
                    "service disabled (build sensor_fusion_msgs first)")

            # ── Keyframe service client ─────────────────────────────────
            self._kf_client = None
            if GetKeyframesInROI is not None:
                self._kf_client = self.create_client(
                    GetKeyframesInROI, str(p("keyframe_service")),
                    callback_group=self._reentrant_group)

            # ── Health check (non-blocking, self-cancelling) ──────────
            self._sam_healthy = None
            self._health_done = False
            self._health_timer = self.create_timer(
                0.5, self._startup_health_check)

            self.get_logger().info(
                f"TsdfFusionNode ready (GRASP mode) "
                f"(voxel={self._voxel_size}m, trunc={self._sdf_trunc}m, "
                f"dbscan_eps={self._dbscan_eps}m, "
                f"shift={self._hit_shift}m)")

        def _init_preview_mode(self, p):
            """Initialise the timer-driven scene-preview path.

            No SAM client, no TriggerGraspFusion service — just a periodic
            loop that fetches all keyframes and integrates them.
            """
            self._sam = None
            self._sam_healthy = None
            self._health_done = True
            self._health_timer = None

            # Preview-specific parameters.
            self._preview_rate_hz = float(p("preview_rate_hz"))
            preview_output_topic = str(p("preview_output_topic"))
            self._preview_enable_bbox_crop = bool(p("preview_enable_bbox_crop"))
            self._preview_enable_dbscan = bool(p("preview_enable_dbscan_cleanup"))

            if self._preview_enable_bbox_crop:
                self._preview_bbox_min = np.asarray(
                    p("preview_workspace_bbox_min"), dtype=np.float64).reshape(3)
                self._preview_bbox_max = np.asarray(
                    p("preview_workspace_bbox_max"), dtype=np.float64).reshape(3)
            else:
                self._preview_bbox_min = None
                self._preview_bbox_max = None

            # ── Publisher (replaces the legacy fusion's output topic) ────
            self._pub = self.create_publisher(
                PointCloud2, preview_output_topic, 10)

            # ── GetAllKeyframes service client ──────────────────────────
            self._kf_all_client = None
            if GetAllKeyframes is not None:
                self._kf_all_client = self.create_client(
                    GetAllKeyframes,
                    str(p("preview_keyframe_service")),
                    callback_group=self._reentrant_group)
            else:
                self.get_logger().warn(
                    "sensor_fusion_msgs not available — GetAllKeyframes "
                    "client disabled (build sensor_fusion_msgs first)")

            # ── Preview timer ───────────────────────────────────────────
            period = 1.0 / max(self._preview_rate_hz, 1e-3)
            self._preview_timer = self.create_timer(
                period, self._preview_tick,
                callback_group=self._reentrant_group)

            # ── In-flight guard — prevents concurrent GetAllKeyframes calls
            #     piling up when the service response serialisation is slow
            #     (100 keyframes × multi-MB images can take several seconds).
            self._preview_fetch_in_flight = False

            self.get_logger().info(
                f"TsdfFusionNode ready (PREVIEW mode) "
                f"(rate={self._preview_rate_hz} Hz, "
                f"output={preview_output_topic}, "
                f"bbox_crop={self._preview_enable_bbox_crop}, "
                f"dbscan={self._preview_enable_dbscan})")

        # ── Health check ───────────────────────────────────────────────

        def _startup_health_check(self):
            """Ping MobileSAM once at startup (non-fatal if down)."""
            if self._health_done:
                return
            self._health_done = True
            # Cancel the timer so this only runs once.
            if self._health_timer is not None:
                self._health_timer.cancel()
                self._health_timer = None
            try:
                ok = self._sam.health()
                self._sam_healthy = ok
                if ok:
                    self.get_logger().info("MobileSAM server reachable.")
                else:
                    self.get_logger().warn(
                        "MobileSAM server NOT reachable — fusion will fail at "
                        "trigger time until the server is up.")
            except Exception as exc:
                self._sam_healthy = False
                self.get_logger().warn(f"SAM health check error: {exc}")

        # ── Service handler ────────────────────────────────────────────

        def _handle_trigger(self, request, response):
            t0 = time.monotonic()
            hit = np.array([
                request.hit_point.x,
                request.hit_point.y,
                request.hit_point.z,
            ], dtype=np.float64)
            camera_id = str(request.camera_id)
            radius = float(request.roi_radius) if request.roi_radius > 0 \
                else self._roi_radius

            self.get_logger().info(
                f"TriggerGraspFusion: hit={hit.tolist()}, cam={camera_id!r}, "
                f"radius={radius:.3f}m")

            # 1. Query keyframes.
            keyframes = self._fetch_keyframes(hit, radius * 1.5)
            if keyframes is None:
                # Service call failed.
                response.success = False
                response.num_points = 0
                response.processing_time_ms = (time.monotonic() - t0) * 1000.0
                response.message = "keyframe buffer service unavailable"
                response.object_cloud = _build_xyzrgb_cloud(
                    np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8),
                    self._make_header())
                return response

            self.get_logger().info(f"  Got {len(keyframes)} keyframes.")

            # 2. Determine the click camera's pose for the inward shift.
            pose_click = self._pose_for_camera(keyframes, camera_id, hit)

            # 3. Run fusion.
            result = fuse_object_cloud(
                keyframes=keyframes,
                hit_point_3d=hit,
                K_click=None,
                pose_click=pose_click,
                sam_segment_fn=self._sam,
                voxel_size=self._voxel_size,
                sdf_trunc=self._sdf_trunc,
                dbscan_eps=self._dbscan_eps,
                dbscan_min_points=self._dbscan_min,
                hit_point_shift=self._hit_shift,
                mask_dilation=self._mask_dilation,
                depth_splat_radius_px=self._depth_splat,
                depth_hole_fill_px=self._depth_hole_fill,
            )

            elapsed_ms = (time.monotonic() - t0) * 1000.0

            # 4. Build + publish the cloud.
            header = self._make_header()
            cloud_msg = _build_xyzrgb_cloud(result.xyz, result.rgb, header)

            if result.success and result.xyz.shape[0] > 0:
                self._pub.publish(cloud_msg)

            response.success = bool(result.success and result.xyz.shape[0] > 0)
            response.object_cloud = cloud_msg
            response.num_points = int(result.xyz.shape[0])
            response.processing_time_ms = float(elapsed_ms)
            response.message = result.message

            self.get_logger().info(
                f"  Fusion done: success={response.success}, "
                f"points={response.num_points}, "
                f"time={elapsed_ms:.1f}ms — {result.message}")
            return response

        # ── Helpers ────────────────────────────────────────────────────

        def _make_header(self):
            h = Header()
            h.stamp = self.get_clock().now().to_msg()
            h.frame_id = self._world_frame
            return h

        def _fetch_keyframes(self, center: np.ndarray, radius: float
                             ) -> Optional[List[Keyframe]]:
            """Call ``GetKeyframesInROI`` and deserialise the result."""
            if self._kf_client is None:
                self.get_logger().error("Keyframe service client not created.")
                return None
            if not self._kf_client.service_is_ready():
                # Wait briefly.
                if not self._kf_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error(
                        "Keyframe buffer service not available.")
                    return None

            req = GetKeyframesInROI.Request()
            req.center = Point(
                x=float(center[0]), y=float(center[1]), z=float(center[2]))
            req.radius = float(radius)

            try:
                future = self._kf_client.call_async(req)
                start = time.monotonic()
                while not future.done():
                    if time.monotonic() - start > 10.0:
                        self.get_logger().error(
                            "Keyframe service call timed out.",
                            throttle_duration_sec=10.0)
                        return None
                    time.sleep(0.05)
            except Exception as exc:
                self.get_logger().error(f"Keyframe service call failed: {exc}")
                return None

            resp = future.result()
            keyframes = [_keyframe_from_msg(m) for m in resp.keyframes]
            return keyframes

        def _fetch_all_keyframes(self) -> Optional[List[Keyframe]]:
            """Call ``GetAllKeyframes`` and deserialise the result.

            Used by the preview timer.  Returns ``None`` on service failure,
            or a (possibly empty) list of keyframes on success.
            """
            if self._kf_all_client is None:
                self.get_logger().error(
                    "GetAllKeyframes client not created.", throttle_duration_sec=10.0)
                return None
            if not self._kf_all_client.service_is_ready():
                if not self._kf_all_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error(
                        "GetAllKeyframes service not available.",
                        throttle_duration_sec=10.0)
                    return None

            req = GetAllKeyframes.Request()
            # Empty camera_id → all cameras.
            req.camera_id = ""

            try:
                future = self._kf_all_client.call_async(req)
                start = time.monotonic()
                while not future.done():
                    if time.monotonic() - start > 10.0:
                        self.get_logger().error(
                            "GetAllKeyframes call timed out.",
                            throttle_duration_sec=10.0)
                        return None
                    time.sleep(0.05)
            except Exception as exc:
                self.get_logger().error(
                    f"GetAllKeyframes call failed: {exc}",
                    throttle_duration_sec=10.0)
                return None

            resp = future.result()
            keyframes = [_keyframe_from_msg(m) for m in resp.keyframes]
            return keyframes

        # ── Preview timer callback ─────────────────────────────────────

        def _preview_tick(self):
            """One cycle of scene-preview fusion (timer-driven)."""
            # ── In-flight guard: if a previous GetAllKeyframes call is still
            #     waiting for the slow service response serialisation, skip
            #     this tick to avoid piling up concurrent calls.
            if self._preview_fetch_in_flight:
                self.get_logger().debug(
                    "Preview: skipping tick — previous GetAllKeyframes still in flight")
                return

            t0 = time.monotonic()

            self._preview_fetch_in_flight = True
            try:
                keyframes = self._fetch_all_keyframes()
            finally:
                self._preview_fetch_in_flight = False
            if keyframes is None:
                # Service error already logged (throttled).
                return

            if len(keyframes) == 0:
                self.get_logger().debug(
                    "Preview: no keyframes yet — waiting for data.")
                return

            result = fuse_scene_preview(
                keyframes=keyframes,
                voxel_size=self._voxel_size,
                sdf_trunc=self._sdf_trunc,
                workspace_bbox_min=self._preview_bbox_min,
                workspace_bbox_max=self._preview_bbox_max,
                dbscan_eps=self._dbscan_eps,
                dbscan_min_points=self._dbscan_min,
                enable_dbscan_cleanup=self._preview_enable_dbscan,
                depth_splat_radius_px=self._depth_splat,
                depth_hole_fill_px=self._depth_hole_fill,
            )

            elapsed_ms = (time.monotonic() - t0) * 1000.0

            if result.success and result.xyz.shape[0] > 0:
                header = self._make_header()
                cloud_msg = _build_xyzrgb_cloud(result.xyz, result.rgb, header)
                self._pub.publish(cloud_msg)
                self.get_logger().info(
                    f"Preview: fused {result.xyz.shape[0]} points from "
                    f"{len(keyframes)} keyframes in {elapsed_ms:.0f}ms")
            else:
                self.get_logger().info(
                    f"Preview: NO GEOMETRY (success={result.success}, "
                    f"points={result.xyz.shape[0]}, msg='{result.message}') "
                    f"from {len(keyframes)} keyframes in {elapsed_ms:.0f}ms")

        @staticmethod
        def _pose_for_camera(keyframes: List[Keyframe], camera_id: str,
                             hit: np.ndarray) -> Optional[np.ndarray]:
            """Pick the pose of the camera that observed the click.

            Prefers a keyframe whose ``camera_id`` matches *camera_id* and
            whose camera is closest to the hit point.  Falls back to the
            nearest keyframe overall.
            """
            if len(keyframes) == 0:
                return None

            def _dist(kf):
                return float(np.linalg.norm(kf.translation - hit))

            preferred = [kf for kf in keyframes if kf.camera_id == camera_id]
            pool = preferred if preferred else keyframes
            best = min(pool, key=_dist)
            return best.pose

    return TsdfFusionNode


def main(args=None):
    """Entry point for the ``tsdf_fusion_node`` console script.

    Uses a MultiThreadedExecutor so that service calls from inside timer
    callbacks can be processed on a different thread.
    """
    (rclpy, *_rest) = _import_ros()
    rclpy.init(args=args)
    NodeClass = create_node()
    node = NodeClass()
    try:
        from rclpy.executors import MultiThreadedExecutor

        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
