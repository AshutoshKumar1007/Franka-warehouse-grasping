"""
ROS grasp generation node.
"""

from __future__ import annotations

import threading
import time 

import rclpy
from transforms3d.quaternions import mat2quat
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)


from sensor_msgs.msg import PointCloud2
from franka_interfaces.msg import GraspCandidate

from .utils import pointcloud2_to_numpy
from shared.protocol import InferenceRequest
from .zmq_client import ZMQInferenceClient
from .grasp_selector import GraspSelector
from .visualization import GraspVisualizer


class GraspNode(Node):
    
    def __init__(self):
        super().__init__("grasp_node")
        
        self.declare_parameter("input_topic", "/preprocessing/points")
        self.declare_parameter("output_topic", "/selected_grasp")
        self.declare_parameter("server_host", "localhost")
        self.declare_parameter("server_port", 5555)
        self.declare_parameter("server_timeout_ms", 8000)
        self.declare_parameter("server_max_retries", 0)
        self.declare_parameter("max_grasps", 50)
        self.declare_parameter("score_threshold", 0.10)
        
        # Read params 
        topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        host  = self.get_parameter("server_host").value
        port  = self.get_parameter("server_port").value
        self.server_timeout_ms = self.get_parameter("server_timeout_ms").value
        self.server_max_retries = self.get_parameter("server_max_retries").value
        self.max_grasps = self.get_parameter("max_grasps").value
        self.score_threshold = self.get_parameter("score_threshold").value
        
        #Pipeline
        self.client = ZMQInferenceClient(
            host,
            port,
            timeout=self.server_timeout_ms,
        )
        
        self.selector = GraspSelector()
        self.visualizer = GraspVisualizer(self)
        
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.sub = self.create_subscription(
            PointCloud2,
            topic,
            self.cloud_cb,
            qos
        )
        
        self.grasp_pub = self.create_publisher(
            GraspCandidate,
            output_topic,
            qos
        )
        
        self.get_logger().info(
            f"Grasp node initialized."
        )

        # Fair blocking/releasing: acquire non-blocking, always
        # release in `finally`. A plain bool worked here only because
        # rclpy's default executor runs callbacks on one thread — the
        # Lock makes the mutual-exclusion intent explicit and keeps
        # this safe if the executor ever changes (MultiThreadedExecutor,
        # a reentrant callback group, etc).
        self.busy_lock = threading.Lock()

    def cloud_cb(self, cloud_msg: PointCloud2):
        """ 
        PointCloud2 -> Numpy
        """
        if not self.busy_lock.acquire(blocking=False):
            # A point cloud is already being processed. We deliberately
            # DROP this one rather than queue it — by the time we'd get
            # to it, a fresher cloud will already be available, and the
            # arm only ever wants the latest grasp candidate, not a
            # backlog of stale ones.
            self.get_logger().debug(
                "Grasp node busy — dropping this point cloud."
            )
            return
        try:
            points = pointcloud2_to_numpy(cloud_msg)
            
            if len(points) == 0:
                self.get_logger().warn("Received empty point cloud.")
                return

            #
            # Build request
            #
            request = InferenceRequest(
                frame_id = cloud_msg.header.frame_id,
                timestamp = time.time(),
                
                point_cloud = points,
                max_grasps = self.max_grasps,
            )
            
            response = self.client.infer(
                request,
                max_retries=self.server_max_retries,
            )
            
            if response is None:
                
                self.get_logger().warn(
                    f"Inference server did not respond within "
                    f"{self.client.last_latency_ms:.0f} ms "
                    f"(timeout={self.server_timeout_ms} ms). If this "
                    f"repeats in tight, regular intervals, the server "
                    f"is behind, not down — consider raising "
                    f"server_timeout_ms."
                )
                return
            self.get_logger().debug(
                f"Inference round trip: "
                f"{self.client.last_latency_ms:.0f} ms."
            )
            if not response.ok():
                self.get_logger().warn(
                    response.message
                )
                return
            #
            # Select best grasp
            #
            best = self.selector.select(
                response.result
            )

            #
            # Visualize all candidates
            #
            self.visualizer.publish(
                response.result
            )

            if best is None:
                self.get_logger().warn(
                    "No valid grasp selected."
                )
                return

            self.get_logger().info(
                f"Best grasp "
                f"score={best.score:.3f}, "
                f"width={best.width:.3f} m"
            )

            #
            # Convert rotation matrix -> quaternion
            #
            qw, qx, qy, qz = mat2quat(
                best.rotation
            )

            #
            # Build GraspCandidate message
            #
            grasp_msg = GraspCandidate()

            #
            # Preserve original frame and timestamp
            #
            grasp_msg.header = cloud_msg.header

            #
            # Position
            #
            grasp_msg.pose.position.x = float(best.position[0])
            grasp_msg.pose.position.y = float(best.position[1])
            grasp_msg.pose.position.z = float(best.position[2])

            #
            # Orientation
            #
            grasp_msg.pose.orientation.x = float(qx)
            grasp_msg.pose.orientation.y = float(qy)
            grasp_msg.pose.orientation.z = float(qz)
            grasp_msg.pose.orientation.w = float(qw)

            #
            # Gripper width
            #
            grasp_msg.width = float(best.width)

            #
            # Confidence
            #
            grasp_msg.score = float(best.score)

            #
            # Publish
            #
            self.grasp_pub.publish(
                grasp_msg
            )

            self.get_logger().info(
                f"Published grasp candidate "
                f"(score={best.score:.3f}, "
                f"frame={grasp_msg.header.frame_id})"
            )
        finally:
            self.busy_lock.release()
            
            
def main():
    rclpy.init()
    node = GraspNode()
    
    try: 
        rclpy.spin(node)
    
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    
    main()