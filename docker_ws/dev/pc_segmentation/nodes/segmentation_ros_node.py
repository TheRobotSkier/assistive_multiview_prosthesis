#!/usr/bin/env python3
"""
InterObject3D ROS 2 segmentation node.

Workflow:
  1. User places seed point(s) in RViz using 'Publish Point' tool → /segmentation/add_point
  2. User calls /segmentation/trigger service → segmentation runs, result latched
  3. /segmented_object_cloud is available to all subscribers (TRANSIENT_LOCAL)
  4. /segmentation/clear_output resets seeds and clears latched result

Topics:
  Sub: /fused_pointcloud              sensor_msgs/PointCloud2
  Sub: /segmentation/add_point        geometry_msgs/PointStamped
  Pub: /segmented_object_cloud        sensor_msgs/PointCloud2  (TRANSIENT_LOCAL / latched)
  Pub: /segmentation/seed_markers     visualization_msgs/MarkerArray

Services:
  /segmentation/trigger               std_srvs/Trigger
  /segmentation/clear_output          std_srvs/Trigger

InterObject3D integration TODO:
  _run_segmentation raises NotImplementedError until the inference API is confirmed.
  Steps when ready:
    1. Convert PointCloud2 → np.ndarray (N,3) via sensor_msgs_py.point_cloud2.read_points
    2. Call model inference with seed coordinates
    3. Convert binary mask back to PointCloud2
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger

LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        self._latest_cloud: PointCloud2 | None = None
        self._seeds: list[tuple[float, float, float]] = []

        self.create_subscription(PointCloud2, '/fused_pointcloud', self._on_cloud, 1)
        self.create_subscription(PointStamped, '/segmentation/add_point',
                                 self._on_seed, 10)

        self._pub_seg = self.create_publisher(
            PointCloud2, '/segmented_object_cloud', LATCHED_QOS)
        self._pub_markers = self.create_publisher(
            MarkerArray, '/segmentation/seed_markers', 10)

        self.create_service(Trigger, '/segmentation/trigger', self._svc_trigger)
        self.create_service(Trigger, '/segmentation/clear_output', self._svc_clear)

        self.get_logger().info(
            'Segmentation node ready. Place seeds with /segmentation/add_point, '
            'then call /segmentation/trigger.')

    def _on_cloud(self, msg: PointCloud2):
        self._latest_cloud = msg

    def _on_seed(self, msg: PointStamped):
        pt = (msg.point.x, msg.point.y, msg.point.z)
        self._seeds.append(pt)
        self.get_logger().info(f'Seed added: {pt}. Total seeds: {len(self._seeds)}')
        self._publish_seed_markers()

    def _svc_trigger(self, _req, resp: Trigger.Response):
        if self._latest_cloud is None:
            resp.success = False
            resp.message = 'No pointcloud received yet.'
            return resp
        if not self._seeds:
            resp.success = False
            resp.message = 'No seeds placed.'
            return resp
        try:
            result_cloud = self._run_segmentation(self._latest_cloud, self._seeds)
            self._pub_seg.publish(result_cloud)
            resp.success = True
            resp.message = f'Segmented with {len(self._seeds)} seeds.'
        except NotImplementedError:
            resp.success = False
            resp.message = 'InterObject3D inference not yet integrated.'
        except Exception as e:
            resp.success = False
            resp.message = str(e)
        return resp

    def _svc_clear(self, _req, resp: Trigger.Response):
        self._seeds = []
        self._publish_seed_markers()
        empty = PointCloud2()
        empty.header.frame_id = 'camera_link'
        self._pub_seg.publish(empty)
        resp.success = True
        resp.message = 'Seeds and segmentation cleared.'
        return resp

    def _run_segmentation(self, cloud: PointCloud2,
                          seeds: list[tuple[float, float, float]]) -> PointCloud2:
        """Call InterObject3D inference. Returns segmented PointCloud2.

        TODO: integrate actual InterObject3D inference here:
          1. pts = np.array(list(sensor_msgs_py.point_cloud2.read_points(cloud, skip_nans=True)))[:, :3]
          2. mask = model.infer(pts, seeds)
          3. filtered = pts[mask]
          4. return sensor_msgs_py.point_cloud2.create_cloud_xyz32(cloud.header, filtered)
        """
        raise NotImplementedError('InterObject3D inference not yet integrated.')

    def _publish_seed_markers(self):
        ma = MarkerArray()
        # First add a DELETE_ALL marker to clear old markers
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)
        for i, (x, y, z) in enumerate(self._seeds):
            m = Marker()
            m.header.frame_id = 'camera_link'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.scale.x = m.scale.y = m.scale.z = 0.03
            m.color.r = 1.0
            m.color.a = 1.0
            ma.markers.append(m)
        self._pub_markers.publish(ma)


def main():
    rclpy.init()
    node = SegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
