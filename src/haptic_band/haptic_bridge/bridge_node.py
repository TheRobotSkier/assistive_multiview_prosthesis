"""
haptic_bridge/bridge_node.py

Bluetooth bridge between ROS 2 and a single Vibro8 haptic-feedback device.

The Vibro8 drives 8 ERM vibro motors.  The bridge subscribes to

    /haptic_band/motors  (std_msgs/Float32MultiArray, 8 elements, 0.0–100.0)

and forwards the intensities to the hardware using the VBA command:

    >VBA;b1b2b3b4b5b6b7b8<   (14 bytes, uint8 per motor, 0–100)

On first successful connection the bridge sends a short "buzz buzz" pulse on
all 8 motors to confirm hardware is live before entering normal operation.
"""

import os
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray

# ── Protocol constants ────────────────────────────────────────────────────────
_BT_CHANNEL = 1

# Disable automatic Michelangelo prosthetic-hand search: >SH;0<
_CMD_DISABLE_MICH = bytes([0x3E, 0x53, 0x48, 0x3B, 0x00, 0x3C])

_VBA_PREFIX = b'>VBA;'  # followed by 8 uint8 intensity bytes then b'<'
_VBA_SUFFIX = b'<'

_RECONNECT_DELAY_S = 3.0
_NUM_MOTORS = 8

# Buzz-on-connect: two short pulses sent to all motors after first connection
_BUZZ_INTENSITY = 80   # % (0–100)
_BUZZ_ON_S      = 0.2  # seconds per pulse
_BUZZ_OFF_S     = 0.15 # seconds between / after pulses
_BUZZ_PULSES    = 2


def build_vba(intensities: list[int]) -> bytes:
    """Build >VBA;b1..b8< command from a list of 8 integers (0-100)."""
    return _VBA_PREFIX + bytes(intensities) + _VBA_SUFFIX


def _normalise_bt_addr(addr: str) -> str:
    """Accept either '842E1409E14E' or '84:2E:14:09:E1:4E' and return the colon form."""
    clean = addr.replace(':', '').upper()
    if len(clean) != 12:
        raise ValueError(f'Invalid Bluetooth address: {addr!r}')
    return ':'.join(clean[i:i+2] for i in range(0, 12, 2))


# ── Per-device Bluetooth connection ──────────────────────────────────────────

class BtDevice:
    """Manages a Bluetooth Classic RFCOMM connection to one Vibro8."""

    def __init__(self, address: str, label: str, logger):
        self._address = address
        self._label = label
        self._log = logger
        self._first_connect = True

        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        self._thread = threading.Thread(target=self._connect_loop, daemon=True)
        self._thread.start()

    def send(self, data: bytes) -> bool:
        """Send bytes; returns False if not connected."""
        with self._lock:
            if self._sock is None:
                return False
            try:
                self._sock.sendall(data)
                return True
            except OSError as exc:
                self._log.warning(f'[{self._label}] send failed: {exc}')
                self._close_locked()
                return False

    def stop(self):
        self._stop_event.set()
        with self._lock:
            self._close_locked()

    def is_connected(self) -> bool:
        with self._lock:
            return self._sock is not None

    # ------------------------------------------------------------------
    def _close_locked(self):
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _buzz(self, sock: socket.socket):
        """Send two short buzz pulses on all motors to confirm connection."""
        on_cmd  = build_vba([_BUZZ_INTENSITY] * 8)
        off_cmd = build_vba([0] * 8)
        for _ in range(_BUZZ_PULSES):
            sock.sendall(on_cmd)
            time.sleep(_BUZZ_ON_S)
            sock.sendall(off_cmd)
            time.sleep(_BUZZ_OFF_S)

    def _connect_loop(self):
        attempt = 0
        while not self._stop_event.is_set():
            with self._lock:
                already_connected = self._sock is not None
            if already_connected:
                time.sleep(0.5)
                continue

            attempt += 1
            self._log.info(f'[{self._label}] Connecting to {self._address} … (attempt {attempt})')
            try:
                sock = socket.socket(
                    socket.AF_BLUETOOTH,
                    socket.SOCK_STREAM,
                    socket.BTPROTO_RFCOMM,
                )
                sock.settimeout(10.0)
                sock.connect((self._address, _BT_CHANNEL))
                sock.settimeout(None)

                sock.sendall(_CMD_DISABLE_MICH)

                if self._first_connect:
                    self._first_connect = False
                    self._log.info(f'[{self._label}] Connected. Sending connection buzz …')
                    self._buzz(sock)
                else:
                    self._log.info(f'[{self._label}] Reconnected.')

                attempt = 0
                with self._lock:
                    self._sock = sock
            except OSError as exc:
                self._log.warning(
                    f'[{self._label}] Connection failed: {exc}. '
                    f'Retrying in {_RECONNECT_DELAY_S}s …'
                )
                time.sleep(_RECONNECT_DELAY_S)


# ── ROS 2 node ───────────────────────────────────────────────────────────────

class HapticBridgeNode(Node):
    def __init__(self):
        super().__init__('haptic_bridge_node')

        addr1 = _normalise_bt_addr(os.environ.get('HAPTIC_BT_ADDR1', '842E1409E14E'))
        self.get_logger().info(f'Device address: {addr1}')
        self._dev1 = BtDevice(addr1, 'dev1', self.get_logger())

        self._conn_status_pub = self.create_publisher(
            Bool,
            '/haptic_band/connection_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._prev_connected = self._dev1.is_connected()
        self._publish_conn_status(self._prev_connected)
        self.create_timer(0.1, self._check_and_publish_conn_status)

        self._sub = self.create_subscription(
            Float32MultiArray,
            '/haptic_band/motors',
            self._on_motors,
            10,
        )
        self.get_logger().info(
            'Subscribed to /haptic_band/motors '
            '(Float32MultiArray with 8 values, 0.0-100.0)'
        )

    # ------------------------------------------------------------------
    def _publish_conn_status(self, connected: bool) -> None:
        msg = Bool()
        msg.data = connected
        self._conn_status_pub.publish(msg)
        self.get_logger().info(
            f'Haptic connection status: {"connected" if connected else "disconnected"}'
        )

    def _check_and_publish_conn_status(self) -> None:
        connected = self._dev1.is_connected()
        if connected != self._prev_connected:
            self._prev_connected = connected
            self._publish_conn_status(connected)

    def _on_motors(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) != _NUM_MOTORS:
            self.get_logger().warn(
                f'Expected {_NUM_MOTORS} motor values, got {len(data)}. Ignoring.'
            )
            return

        intensities = [max(0, min(100, round(v))) for v in data]

        if not self._dev1.send(build_vba(intensities)):
            self.get_logger().debug('dev1 not connected, command dropped.')

    def destroy_node(self):
        self._dev1.stop()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HapticBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

