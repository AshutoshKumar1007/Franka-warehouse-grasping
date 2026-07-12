# """
# franka_perception / perception_node.py

# Single job: take raw camera topics (image, depth, points, camera_info) and
# publish them on a stable internal API (/perception/*), with the point cloud
# transformed into a known, stable robot frame.
# """

# import numpy as np
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import qos_profile_sensor_data
# from sensor_msgs.msg import Image, PointCloud2, CameraInfo
# from sensor_msgs_py import point_cloud2 as pc2
# from tf2_ros import Buffer, TransformListener
# from transforms3d.quaternions import quat2mat
# import rclpy.time
# import rclpy.duration


# class PerceptionNode(Node):
#     def __init__(self):
#         super().__init__('perception_node')

#         self.declare_parameter('input_prefix', '/rgbd_camera')
#         self.declare_parameter('output_prefix', '/perception')
#         self.declare_parameter('target_frame', 'fr3_link0')
#         self.declare_parameter('tf_timeout_sec', 1.0)

#         in_prefix = self.get_parameter('input_prefix').value
#         out_prefix = self.get_parameter('output_prefix').value
#         self.target_frame = self.get_parameter('target_frame').value
#         self.tf_timeout = self.get_parameter('tf_timeout_sec').value

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.points_sub = self.create_subscription(
#             PointCloud2, f'{in_prefix}/points', self.points_cb, qos_profile_sensor_data)
#         self.points_pub = self.create_publisher(
#             PointCloud2, f'{out_prefix}/points', qos_profile_sensor_data)

#         self.image_sub = self.create_subscription(
#             Image, f'{in_prefix}/image', self._relay(f'{out_prefix}/image', Image), qos_profile_sensor_data)
#         self.depth_sub = self.create_subscription(
#             Image, f'{in_prefix}/depth_image', self._relay(f'{out_prefix}/depth', Image), qos_profile_sensor_data)
#         self.info_sub = self.create_subscription(
#             CameraInfo, f'{in_prefix}/camera_info', self._relay(f'{out_prefix}/camera_info', CameraInfo), qos_profile_sensor_data)

#         self.get_logger().info(
#             f"perception_node up: '{in_prefix}/*' -> '{out_prefix}/*' "
#             f"(points transformed into '{self.target_frame}')")

#     def _relay(self, out_topic, msg_type):
#         pub = self.create_publisher(msg_type, out_topic, qos_profile_sensor_data)

#         def cb(msg):
#             pub.publish(msg)
#         return cb

#     def _lookup(self, source_frame, stamp):
#         try:
#             return self.tf_buffer.lookup_transform(
#                 self.target_frame, source_frame,
#                 rclpy.time.Time.from_msg(stamp),
#                 timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
#         except Exception:
#             return self.tf_buffer.lookup_transform(
#                 self.target_frame, source_frame, rclpy.time.Time())

#     def points_cb(self, msg: PointCloud2):
#         try:
#             tf = self._lookup(msg.header.frame_id, msg.header.stamp)
#         except Exception as e:
#             self.get_logger().warn(
#                 f'no transform {msg.header.frame_id} -> {self.target_frame} yet: {e}')
#             return

#         # Extract xyz explicitly (robust to padding / rgb fields in the cloud).
#         pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
#         if pts.size == 0:
#             return

#         # Build 4x4 from the TF and apply it.
#         t = tf.transform.translation
#         q = tf.transform.rotation
#         R = quat2mat([q.w, q.x, q.y, q.z])
#         xyz = (R @ pts.T).T + np.array([t.x, t.y, t.z])

#         header = msg.header
#         header.frame_id = self.target_frame
#         out = pc2.create_cloud_xyz32(header, xyz.astype(np.float32))
#         self.points_pub.publish(out)


# def main():
#     rclpy.init()
#     node = PerceptionNode()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
"""
franka_perception / perception_node.py

Single job: take raw camera topics (image, depth, points, camera_info) and
publish them on a stable internal API (/perception/*), with the point cloud
transformed into a known, stable robot frame.
"""

import numpy as np

import rclpy
import rclpy.duration
import rclpy.time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2 as pc2

from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Header

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """Convert quaternion (x,y,z,w) to a 3x3 rotation matrix."""

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)]
    ], dtype=np.float32)


class PerceptionNode(Node):

    def __init__(self):
        super().__init__("perception_node")

        self.declare_parameter("input_prefix", "/rgbd_camera")
        self.declare_parameter("output_prefix", "/perception")
        self.declare_parameter("target_frame", "fr3_link0")
        self.declare_parameter("tf_timeout_sec", 1.0)

        in_prefix = self.get_parameter("input_prefix").value
        out_prefix = self.get_parameter("output_prefix").value

        self.target_frame = self.get_parameter("target_frame").value
        self.tf_timeout = self.get_parameter("tf_timeout_sec").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        PIPELINE_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.points_sub = self.create_subscription(
            PointCloud2,
            f"{in_prefix}/points",
            self.points_cb,
            qos_profile_sensor_data, #BEST_EFFORT
        )

        self.points_pub = self.create_publisher(
            PointCloud2,
            f"{out_prefix}/points",
            PIPELINE_qos, # RELIABLE
        )

        self.image_sub = self.create_subscription(
            Image,
            f"{in_prefix}/image",
            self._relay(f"{out_prefix}/image", Image),
            qos_profile_sensor_data,
        )

        self.depth_sub = self.create_subscription(
            Image,
            f"{in_prefix}/depth_image",
            self._relay(f"{out_prefix}/depth", Image),
            qos_profile_sensor_data,
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            f"{in_prefix}/camera_info",
            self._relay(f"{out_prefix}/camera_info", CameraInfo),
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"perception_node up: '{in_prefix}/*' -> '{out_prefix}/*' "
            f"(points transformed into '{self.target_frame}')"
        )

    def _relay(self, out_topic, msg_type):
        pub = self.create_publisher(
            msg_type,
            out_topic,
            qos_profile_sensor_data,
        )

        def callback(msg):
            pub.publish(msg)

        return callback

    def _lookup_transform(self, source_frame):

        if not self.tf_buffer.can_transform(
            self.target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
        ):
            raise RuntimeError(
                f"Transform {source_frame} -> {self.target_frame} not available yet."
            )

        return self.tf_buffer.lookup_transform(
            self.target_frame,
            source_frame,
            rclpy.time.Time(),
        )

    def points_cb(self, msg: PointCloud2):

        try:
            transform = self._lookup_transform(
                msg.header.frame_id,
            )
        except Exception as e:
            self.get_logger().warn(
                f"No transform from {msg.header.frame_id} "
                f"to {self.target_frame}: {e}"
            )
            return

        points = pc2.read_points_numpy(
            msg,
            field_names=["x", "y", "z"],
            skip_nans=True,
        )

        if len(points) == 0:
            return

        t = transform.transform.translation
        q = transform.transform.rotation

        R = quaternion_to_rotation_matrix(
            q.x,
            q.y,
            q.z,
            q.w,
        )

        transformed = (R @ points.T).T
        transformed += np.array(
            [t.x, t.y, t.z],
            dtype=np.float32,
        )

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.target_frame
        
        cloud = pc2.create_cloud_xyz32(
            header,
            transformed.astype(np.float32),
        )

        self.points_pub.publish(cloud)


def main():
    rclpy.init()

    node = PerceptionNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()