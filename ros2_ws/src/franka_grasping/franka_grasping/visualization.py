"""
RViz visualization utilities.

This module publishes grasp candidates as MarkerArray messages.

Visualization is completely independent of inference and planning.
"""

from __future__ import annotations

import numpy as np

from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray

from geometry_msgs.msg import Point

from shared.grasp_type import Grasp, GraspResult

class GraspVisualizer:
    
    def __init__(self, node : Node):
        
        self.publisher = node.create_publisher(
            MarkerArray,
            "/grasping/visualization",
            1
        )
        self.frame_id = "fr3_link0"
        
    def publish(
        self,
        result : GraspResult,
    ): 
        
        markers = MarkerArray()
        for idx, grasp in enumerate(result.grasps):
            markers.markers.extend(
                self._build_grasp_markers(
                    grasp,
                    idx
                )
            )
        self.publisher.publish(markers)
        
    def clear(self):
        
        marker = Marker()
        marker.action = Marker.DELETEALL
        
        arr = MarkerArray()
        
        arr.markers.append(marker)
        self.publisher.publish(arr)
    
    def _build_grasp_markers(
        self,
        grasp : Grasp,
        marker_id : int,
    ) -> list[Marker]:
        """ 
        Build RViz markers for a single grasp candidate.
        """
        marker = Marker()
        marker.header.frame_id = self.frame_id
        
        marker.ns = "grasps"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.scale.x = 0.08
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        p0  = Point()

        p0.x = float(grasp.position[0])
        p0.y = float(grasp.position[1])
        p0.z = float(grasp.position[2])

        direction = grasp.rotation[:, 0]
        
        dx = float(direction[0])
        dy = float(direction[1])
        dz = float(direction[2])

        p1 = Point()
        p1.x = p0.x + 0.06 * dx
        p1.y = p0.y + 0.06 * dy
        p1.z = p0.z + 0.06 * dz
        
        marker.points.append(p0)
        marker.points.append(p1)
        
        return [marker] 