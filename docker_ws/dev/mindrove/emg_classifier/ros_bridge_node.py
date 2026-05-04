"""
ROS 2 bridge: publishes live EMG predictions to ROS topics.

Run via spin_in_thread() alongside run_classifier.py main loop.
If rclpy is not installed, importing this module will raise ImportError (expected).

Topics published (TRANSIENT_LOCAL / latched, depth 1):
  /emg/gesture_label     std_msgs/Int32
  /emg/gesture_name      std_msgs/String
  /emg/confidence        std_msgs/Float32
  /emg/proportional      std_msgs/Float32
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float32, Int32, String


_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@dataclass
class EmgState:
    label: int = 0
    name: str = "REST"
    confidence: float = 0.0
    proportional: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)


class EmgRosBridgeNode(Node):
    def __init__(self, state: EmgState):
        super().__init__('emg_ros_bridge')
        self._state = state
        self._pub_label = self.create_publisher(Int32,   '/emg/gesture_label', _LATCHED_QOS)
        self._pub_name  = self.create_publisher(String,  '/emg/gesture_name',  _LATCHED_QOS)
        self._pub_conf  = self.create_publisher(Float32, '/emg/confidence',    _LATCHED_QOS)
        self._pub_prop  = self.create_publisher(Float32, '/emg/proportional',  _LATCHED_QOS)
        self.create_timer(0.05, self._publish)   # 20 Hz

    def _publish(self):
        with self._state.lock:
            label = self._state.label
            name  = self._state.name
            conf  = float(self._state.confidence)
            prop  = float(self._state.proportional)
        self._pub_label.publish(Int32(data=label))
        self._pub_name.publish(String(data=name))
        self._pub_conf.publish(Float32(data=conf))
        self._pub_prop.publish(Float32(data=prop))


def spin_in_thread(state: EmgState) -> threading.Thread:
    """Start ROS spin in a daemon thread. Returns thread handle."""
    def _run():
        rclpy.init()
        node = EmgRosBridgeNode(state)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    t = threading.Thread(target=_run, daemon=True)
    t.start()
    return t
