"""
franka_perception / preprocessing_node.py

Single job: clean a base-frame point cloud for downstream grasp models.
    /perception/points  ->  [crop -> voxel -> optional outlier removal]  ->  /preprocessing/points

Pure NumPy (crop + voxel). SciPy is used only if outlier removal is enabled,
imported lazily so the node has no hard SciPy dependency.
"""

import numpy as np
import rclpy
from rclpy.node import Node
# from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

class PreprocessingNode(Node):
    def __init__(self):
        super().__init__("preprocessing_node")

        self.declare_parameter("input_topic", "/perception/points")
        self.declare_parameter("output_topic", "/preprocessing/points")
        # workspace crop bounds in the robot base frame (fr3_link0), metres
        self.declare_parameter("crop_min", [-0.30, -0.60, -0.05])
        self.declare_parameter("crop_max", [0.90, 0.60, 0.50])
        self.declare_parameter("voxel_size", 0.005)
        self.declare_parameter("remove_outliers", False)
        self.declare_parameter("sor_neighbors", 20)
        self.declare_parameter("sor_std_ratio", 2.0)

        self.crop_min = np.array(self.get_parameter("crop_min").value, dtype=np.float32)
        self.crop_max = np.array(self.get_parameter("crop_max").value, dtype=np.float32)
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.remove_outliers = bool(self.get_parameter("remove_outliers").value)
        self.sor_neighbors = int(self.get_parameter("sor_neighbors").value)
        self.sor_std_ratio = float(self.get_parameter("sor_std_ratio").value)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        PIPELINE_QOS = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        self.sub = self.create_subscription(
            PointCloud2,
            in_topic, 
            self.cloud_cb,
            PIPELINE_QOS
        )
        self.pub = self.create_publisher(
            PointCloud2,
            out_topic, 
            PIPELINE_QOS
        )
        self.get_logger().info(
            f"preprocessing_node up: '{in_topic}' -> '{out_topic}' | "
            f"crop {self.crop_min.tolist()}..{self.crop_max.tolist()}, "
            f"voxel {self.voxel_size} m, outliers={self.remove_outliers}")

    def crop(self, pts):
        m = np.all((pts >= self.crop_min) & (pts <= self.crop_max), axis=1)
        return pts[m]

    def voxel_downsample(self, pts):
        if self.voxel_size <= 0.0:
            return pts
        keys = np.floor(pts / self.voxel_size).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[idx]

    def statistical_outlier_removal(self, pts):
        try:
            from scipy.spatial import cKDTree
        except ImportError:
            self.get_logger().warn("scipy not available; skipping outlier removal")
            return pts
        if len(pts) <= self.sor_neighbors:
            return pts
        tree = cKDTree(pts)
        d, _ = tree.query(pts, k=self.sor_neighbors + 1)
        mean_d = d[:, 1:].mean(axis=1)
        thresh = mean_d.mean() + self.sor_std_ratio * mean_d.std()
        return pts[mean_d < thresh]

    def cloud_cb(self, msg: PointCloud2):
        pts = pc2.read_points_numpy(msg, field_names=["x", "y", "z"], skip_nans=True)
        if len(pts) == 0:
            return
        n0 = len(pts)
        pts = self.crop(pts)
        n1 = len(pts)
        if n1 == 0:
            self.get_logger().warn("all points removed by workspace crop; check bounds")
            return
        pts = self.voxel_downsample(pts)
        n2 = len(pts)
        if self.remove_outliers:
            pts = self.statistical_outlier_removal(pts)
        n3 = len(pts)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id  # already in target frame, set by perception node
        out = pc2.create_cloud_xyz32(
            header, 
            pts.astype(np.float32)
        )
        self.pub.publish(out)
        self.get_logger().info(
            f"{n0} -> crop {n1} -> voxel {n2} -> out {n3}", throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = PreprocessingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
