"""ctypes bridge to libgrasp_preshaping.so.

Provides Python access to the Rust grasp planning pipeline via the
C FFI interface defined in src/grasp_preshaping/include/grasp_preshaping/ffi_types.hpp
and src/grasp_preshaping/src/c_api.rs.

Usage:
    lib = GraspLibrary()
    req = make_request(pose, twist, cloud_np, cameras)
    status, resp, msg = compute(lib, req)
    print(resp["pipeline_time_ms"], "ms")
"""

import ctypes
import os
from typing import Optional

# ---------------------------------------------------------------------------
# FFI struct definitions (must match ffi_types.hpp and c_api.rs exactly)
# ---------------------------------------------------------------------------

class GraspPoseFFI(ctypes.Structure):
    _fields_ = [
        ("px", ctypes.c_double),
        ("py", ctypes.c_double),
        ("pz", ctypes.c_double),
        ("qx", ctypes.c_double),
        ("qy", ctypes.c_double),
        ("qz", ctypes.c_double),
        ("qw", ctypes.c_double),
    ]


class GraspTwistFFI(ctypes.Structure):
    _fields_ = [
        ("lx", ctypes.c_double),
        ("ly", ctypes.c_double),
        ("lz", ctypes.c_double),
        ("ax", ctypes.c_double),
        ("ay", ctypes.c_double),
        ("az", ctypes.c_double),
    ]


class PointCloudViewFFI(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_size_t),
        ("height", ctypes.c_size_t),
        ("point_step", ctypes.c_size_t),
        ("x_off", ctypes.c_size_t),
        ("y_off", ctypes.c_size_t),
        ("z_off", ctypes.c_size_t),
        ("data_ptr", ctypes.POINTER(ctypes.c_uint8)),
        ("data_len", ctypes.c_size_t),
    ]


class CameraPositionFFI(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
    ]


class GraspComputeRequestFFI(ctypes.Structure):
    _fields_ = [
        ("pose", GraspPoseFFI),
        ("twist", GraspTwistFFI),
        ("cloud", PointCloudViewFFI),
        ("cameras", CameraPositionFFI * 4),
        ("n_cameras", ctypes.c_uint32),
    ]


class GraspComputeResponseFFI(ctypes.Structure):
    _fields_ = [
        ("success", ctypes.c_uint8),
        ("closure_amount", ctypes.c_double),
        ("combined_score", ctypes.c_double),
        ("grasp_type", ctypes.c_int32),
        ("alignment_score", ctypes.c_double),
        ("force_closure_score", ctypes.c_double),
        ("contact_count_score", ctypes.c_double),
        ("contact_score", ctypes.c_double),
        ("second_best_combined_score", ctypes.c_double),
        ("second_best_grasp_type", ctypes.c_int32),
        ("thumb_closure", ctypes.c_double),
        ("index_closure", ctypes.c_double),
        ("mrl_closure", ctypes.c_double),
        ("pipeline_time_ms", ctypes.c_uint32),
        ("smc_iterations_used", ctypes.c_uint32),
        ("target_px", ctypes.c_double),
        ("target_py", ctypes.c_double),
        ("target_pz", ctypes.c_double),
        ("wrist_qx", ctypes.c_double),
        ("wrist_qy", ctypes.c_double),
        ("wrist_qz", ctypes.c_double),
        ("wrist_qw", ctypes.c_double),
        ("wrist_rotation_deg", ctypes.c_double),
    ]


# Return codes
GRASP_COMPUTE_OK = 0
GRASP_COMPUTE_INVALID_ARGS = 1
GRASP_COMPUTE_PANIC = 2

# Grasp type IDs
GRASP_TYPE_UNKNOWN = 0
GRASP_TYPE_CYLINDRICAL = 1
GRASP_TYPE_PINCH = 2
GRASP_TYPE_LATERAL = 3

GRASP_TYPE_NAMES = {
    GRASP_TYPE_UNKNOWN: "unknown",
    GRASP_TYPE_CYLINDRICAL: "cylindrical",
    GRASP_TYPE_PINCH: "pinch",
    GRASP_TYPE_LATERAL: "lateral",
}

# ---------------------------------------------------------------------------
# Library loader
# ---------------------------------------------------------------------------

_SO_SEARCH_PATHS = [
    # Inside Docker container (installed via colcon)
    "/prosthesis_ws/install/grasp_preshaping/lib/libgrasp_preshaping.so",
    # Local build
    os.path.join(
        os.path.dirname(__file__),
        "..", "..", "src", "grasp_preshaping", "lib", "libgrasp_preshaping.so",
    ),
    # Another local path variant
    os.path.join(
        os.path.dirname(__file__),
        "..", "..", "src", "grasp_preshaping", "target", "release",
        "libgrasp_preshaping.so",
    ),
]


def _find_so() -> str:
    env_path = os.environ.get("GRASP_PRESHAPING_LIB_PATH", "")
    if env_path and os.path.isfile(env_path):
        return env_path
    for p in _SO_SEARCH_PATHS:
        p = os.path.normpath(p)
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(
        "libgrasp_preshaping.so not found. Searched:\n"
        + "\n".join(f"  {p}" for p in _SO_SEARCH_PATHS)
        + "\nSet GRASP_PRESHAPING_LIB_PATH to override."
    )


class GraspLibrary:
    """Loaded Rust grasp preshaping library."""

    def __init__(self, so_path: Optional[str] = None):
        if so_path is None:
            so_path = _find_so()
        self.so_path = so_path
        self._lib = ctypes.CDLL(so_path)

        # Set up function signatures
        self._api_version = self._lib.grasp_preshaping_api_version
        self._api_version.restype = ctypes.c_uint32
        self._api_version.argtypes = []

        self._compute = self._lib.grasp_preshaping_compute
        self._compute.restype = ctypes.c_int32
        self._compute.argtypes = [
            ctypes.POINTER(GraspComputeRequestFFI),
            ctypes.POINTER(GraspComputeResponseFFI),
            ctypes.c_char_p,
            ctypes.c_size_t,
        ]

    @property
    def api_version(self) -> int:
        return self._api_version()

    def compute(
        self,
        request: GraspComputeRequestFFI,
    ) -> tuple[int, GraspComputeResponseFFI, str]:
        """Call grasp_preshaping_compute.

        Returns (status_code, response_struct, message_string).
        """
        response = GraspComputeResponseFFI()
        buf = ctypes.create_string_buffer(512)

        status = self._compute(
            ctypes.byref(request),
            ctypes.byref(response),
            buf,
            ctypes.sizeof(buf),
        )
        message = buf.value.decode("utf-8", errors="replace")
        return status, response, message


# ---------------------------------------------------------------------------
# Request builder helpers
# ---------------------------------------------------------------------------

def make_pose(px, py, pz, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> GraspPoseFFI:
    return GraspPoseFFI(
        px=float(px), py=float(py), pz=float(pz),
        qx=float(qx), qy=float(qy), qz=float(qz), qw=float(qw),
    )


def make_twist(lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0) -> GraspTwistFFI:
    return GraspTwistFFI(
        lx=float(lx), ly=float(ly), lz=float(lz),
        ax=float(ax), ay=float(ay), az=float(az),
    )


def make_request(
    pose: GraspPoseFFI,
    twist: GraspTwistFFI,
    cloud_np,  # numpy (N,3) float32 array
    cameras: list[tuple[float, float, float]],
) -> GraspComputeRequestFFI:
    """Build a GraspComputeRequestFFI from numpy point cloud and camera list.

    Args:
        pose: Hand pose.
        twist: Hand twist.
        cloud_np: (N, 3) float32 numpy array of XYZ points.
        cameras: List of (x, y, z) camera positions (up to 4).
    """
    import numpy as np

    cloud = np.ascontiguousarray(cloud_np, dtype=np.float32)
    n_points = cloud.shape[0]

    # Build interleaved XYZ buffer (point_step=12, offsets 0,4,8)
    cloud_flat = cloud.reshape(-1)  # already interleaved from (N,3)
    data_ptr = cloud_flat.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
    data_len = cloud_flat.nbytes

    cam_array = (CameraPositionFFI * 4)()
    for i, (cx, cy, cz) in enumerate(cameras[:4]):
        cam_array[i] = CameraPositionFFI(x=float(cx), y=float(cy), z=float(cz))

    return GraspComputeRequestFFI(
        pose=pose,
        twist=twist,
        cloud=PointCloudViewFFI(
            width=n_points,
            height=1,
            point_step=12,
            x_off=0,
            y_off=4,
            z_off=8,
            data_ptr=data_ptr,
            data_len=data_len,
        ),
        cameras=cam_array,
        n_cameras=min(len(cameras), 4),
    )


def response_to_dict(resp: GraspComputeResponseFFI) -> dict:
    """Convert FFI response struct to a plain dict."""
    return {
        "success": bool(resp.success),
        "closure_amount": resp.closure_amount,
        "combined_score": resp.combined_score,
        "grasp_type": resp.grasp_type,
        "grasp_type_name": GRASP_TYPE_NAMES.get(resp.grasp_type, "unknown"),
        "alignment_score": resp.alignment_score,
        "force_closure_score": resp.force_closure_score,
        "contact_count_score": resp.contact_count_score,
        "contact_score": resp.contact_score,
        "second_best_combined_score": resp.second_best_combined_score,
        "second_best_grasp_type": resp.second_best_grasp_type,
        "thumb_closure": resp.thumb_closure,
        "index_closure": resp.index_closure,
        "mrl_closure": resp.mrl_closure,
        "pipeline_time_ms": resp.pipeline_time_ms,
        "smc_iterations_used": resp.smc_iterations_used,
        "target_px": resp.target_px,
        "target_py": resp.target_py,
        "target_pz": resp.target_pz,
        "wrist_qx": resp.wrist_qx,
        "wrist_qy": resp.wrist_qy,
        "wrist_qz": resp.wrist_qz,
        "wrist_qw": resp.wrist_qw,
        "wrist_rotation_deg": resp.wrist_rotation_deg,
    }
