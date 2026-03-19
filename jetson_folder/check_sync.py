#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from collections import deque

def to_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

class SyncChecker(Node):
    def __init__(self):
        super().__init__('sync_checker')

        self.c1 = deque(maxlen=50)
        self.c2 = deque(maxlen=50)
        self.d1 = deque(maxlen=50)
        self.d2 = deque(maxlen=50)

        self.create_subscription(Image, '/cam1/d435_1/color/image_raw', self.cb_c1, 20)
        self.create_subscription(Image, '/cam2/d435_2/color/image_raw', self.cb_c2, 20)
        self.create_subscription(Image, '/cam1/d435_1/depth/image_rect_raw', self.cb_d1, 20)
        self.create_subscription(Image, '/cam2/d435_2/depth/image_rect_raw', self.cb_d2, 20)

    def nearest_ms(self, t, q):
        if not q:
            return None
        return min(abs(t - x) for x in q) * 1000.0

    def cb_c1(self, msg):
        t = to_sec(msg)
        self.c1.append(t)
        dt = self.nearest_ms(t, self.c2)
        if dt is not None:
            self.get_logger().info(f'color nearest dt = {dt:.3f} ms')

    def cb_c2(self, msg):
        t = to_sec(msg)
        self.c2.append(t)
        dt = self.nearest_ms(t, self.c1)
        if dt is not None:
            self.get_logger().info(f'color nearest dt = {dt:.3f} ms')

    def cb_d1(self, msg):
        t = to_sec(msg)
        self.d1.append(t)
        dt = self.nearest_ms(t, self.d2)
        if dt is not None:
            self.get_logger().info(f'depth nearest dt = {dt:.3f} ms')

    def cb_d2(self, msg):
        t = to_sec(msg)
        self.d2.append(t)
        dt = self.nearest_ms(t, self.d1)
        if dt is not None:
            self.get_logger().info(f'depth nearest dt = {dt:.3f} ms')

def main():
    rclpy.init()
    node = SyncChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()