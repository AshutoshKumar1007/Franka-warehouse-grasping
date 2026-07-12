"""
Abstract interface implemented by every grasp generation backend.
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import List, Dict, Any

class BaseGraspModel(ABC):
    """Abstract base class for grasp generation models."""
    
    @abstractmethod
    def load(self):
        """Load the model into memory."""
        pass

    @abstractmethod
    def infer(
        self,
        point_cloud,
        segmentation=None,
        camera_intrinsics=None,
    ) -> List[Dict[str,Any]]:
        """
        Run inference.

        Parameters
        ----------
        point_cloud
            numpy.ndarray (N,3)

        segmentation
            Optional segmentation labels.

        camera_intrinsics
            Optional 3x3 intrinsic matrix.

        Returns
        -------
        List[Dict]

        Canonical grasp representation.

        [
            {
                "score": float,
                "position": [x,y,z],
                "rotation": [[...],[...],[...]],
                "width": float
            },
            ...
        ]
        """
        pass