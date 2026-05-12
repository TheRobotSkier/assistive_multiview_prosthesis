#!/usr/bin/env python3
"""Convert color (BGR8/RGB8) image stream to greyscale (mono8).

Replaces the image_proc/convert node when image_proc is not installed in the
container. Uses cv_bridge + OpenCV for zero-copy-friendly conversion.

Parameters
----------
~input_topic  : str  (default: "color_image_raw")
    Source color image topic (relative to node namespace).
~output_topic : str  (default: "grey_image_raw")
    Destination mono8 image topic (relative to node namespace).
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ColorToGreyNode(Node):
    def __init__(self):
        super().__init__("color_to_grey")

        self.declare_parameter("input_topic", "color_image_raw")
        self.declare_parameter("output_topic", "grey_image_raw")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, output_topic, 10)
        self._sub = self.create_subscription(Image, input_topic, self._cb, 10)
        self.get_logger().info(
            f"color_to_grey: {input_topic} -> {output_topic}"
        )

    def _cb(self, msg: Image):
        try:
            encoding = msg.encoding.lower()
            if encoding in ("bgr8", "bgr"):
                cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                grey = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            else:
                # Treat everything else (rgb8, rgba8, bayer_*, yuv…) as rgb8
                cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                grey = cv2.cvtColor(cv_img, cv2.COLOR_RGB2GRAY)

            out_msg = self._bridge.cv2_to_imgmsg(grey, encoding="mono8")
            out_msg.header = msg.header
            self._pub.publish(out_msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"color_to_grey conversion failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorToGreyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
