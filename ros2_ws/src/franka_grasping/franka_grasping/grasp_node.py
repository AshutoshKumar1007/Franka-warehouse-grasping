"""
ROS grasp generation node.
"""

from __future__ import annotations

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
        self.declare_parameter("max_grasps", 50)
        self.declare_parameter("score_threshold", 0.10)
        
        # Read params 
        topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        host  = self.get_parameter("server_host").value
        port  = self.get_parameter("server_port").value
        self.max_grasps = self.get_parameter("max_grasps").value
        self.score_threshold = self.get_parameter("score_threshold").value
        
        #Pipeline
        self.client = ZMQInferenceClient(host, port)
        
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
        self.busy = False
        
    def cloud_cb(self, cloud_msg: PointCloud2):
        """ 
        PointCloud2 -> Numpy
        """
        if self.busy:
            return
        self.busy = True
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
            
            response = self.client.infer(request)
            
            if response is None:
                
                self.get_logger().warn(
                    "Inference server did not respond."
                )
                return
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
            self.busy = False
            
            
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