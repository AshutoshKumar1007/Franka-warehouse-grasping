"""
Utility functions used throughout the grasping package.

No ROS nodes.
No networking.
No inference.

Only data conversion.
"""

from __future__ import annotations

import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

def pointcloud2_to_numpy(
    msg: PointCloud2,
) -> np.ndarray:
    """
    Convert a ROS PointCloud2 message into an (N,3) float32 array.
    """

    points = pc2.read_points_numpy(
        msg,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )

    return points.astype(np.float32)



def transform_points(
    points: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
) -> np.ndarray:
    """
    Apply rigid transformation.

    points' = R * points + t
    """

    return (R @ points.T).T + t