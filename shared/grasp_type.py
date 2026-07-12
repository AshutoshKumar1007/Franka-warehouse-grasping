"""
Canonical grasp representation.

This module defines the data structures used throughout the ROS grasping
pipeline. Every grasping backend (GraspNet, GSNet, AnyGrasp, etc.) should be
converted into these types before reaching the ROS side.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import numpy as np

@dataclass(slots=True)
class Grasp:
    """ 
    Represents one 6-DoF grasp candidate.
    """
    
    position: np.ndarray # (3,)
    rotation: np.ndarray # (3, 3)
    width: float
    score: float
    
    metadata: dict = field(default_factory=dict)

    def pose_matrix(self) -> np.ndarray:
        """Return the 4x4 homogeneous transformation matrix of the grasp."""
        T = np.eye(4)
        T[:3, :3] = self.rotation
        T[:3, 3] = self.position
        return T

@dataclass(slots=True)
class GraspResult:
    """ 
    Collection of grasp candidates.
    """
    grasps : List[Grasp]
    
    inference_time: float = 0.0
    
    backend : str = ""
    model : str = ""
    
    @staticmethod
    def from_list(
        grasp_list: list[dict],
        inference_time: float = 0.0,
        backend: str = "",
        model: str = "",
    ) -> "GraspResult":

        grasps = [
            Grasp(
                position=np.asarray(g["position"], dtype=np.float32),
                rotation=np.asarray(g["rotation"], dtype=np.float32),
                width=float(g["width"]),
                score=float(g["score"]),
            )
            for g in grasp_list
        ]

        return GraspResult(
            grasps=grasps,
            inference_time=inference_time,
            backend=backend,
            model=model,
        )
    def empty(self) -> bool:
        return len(self.grasps) == 0
    
    @property
    def best(self) -> Grasp | None:
        if self.empty():
            return None
        
        return max(self.grasps, key=lambda g: g.score)
    
    def sort(self):
        self.grasps.sort(
            key = lambda g: g.score, reverse=True
        )
    