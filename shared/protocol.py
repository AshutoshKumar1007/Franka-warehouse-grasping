"""
Shared communication protocol between the ROS grasp node and the
external inference server.

This module contains ONLY serializable dataclasses.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import numpy as np
import pickle

from .grasp_type import Grasp, GraspResult


PROTOCOL_VERSION = 1

# ----------------------------------------------------------------------
# Request
# ----------------------------------------------------------------------

@dataclass(slots=True)
class InferenceRequest:
    """
    Request sent from ROS2 -> inference server.
    """
    version : int = PROTOCOL_VERSION
    
    frame_id : str = 'fr3_link0'
    timestamp : float = 0.0
    
    point_cloud : np.ndarray | None = None # (N, 3)
    segmentation: np.ndarray | None = None
    
    max_grasps : int = 50
    
    def to_bytes(self) -> bytes:
        return pickle.dumps({
        "version": self.version,
        "frame_id": self.frame_id,
        "timestamp": self.timestamp,
        "point_cloud": self.point_cloud,
        "segmentation": self.segmentation,
        "max_grasps": self.max_grasps,
    })
    
    @staticmethod
    def from_bytes(data: bytes) -> InferenceRequest:
        kwargs = pickle.loads(data)
        return InferenceRequest(**kwargs)

# ----------------------------------------------------------------------
# Response
# ----------------------------------------------------------------------

@dataclass(slots=True)  
class InferenceResponse:
    """ 
    Response sent from inference server -> ROS2.
    """
    success : bool
    
    message : str = ""
    
    result : GraspResult | None = None
    
    inference_time_ms : float = 0.0
    
    backend : str = ""
    
    def to_bytes(self) -> bytes:
        return pickle.dumps({
        "success": self.success,
        "message": self.message,
        "result": self.result,
        "inference_time_ms": self.inference_time_ms,
        "backend": self.backend,
    })
    
    @staticmethod
    def from_bytes(data: bytes) -> "InferenceResponse":
        kwargs = pickle.loads(data)
        return InferenceResponse(**kwargs)
    
    def ok(self) -> bool:
        return self.success and self.result is not None and not self.result.empty()
    
# ----------------------------------------------------------------------
# Status
# ----------------------------------------------------------------------

@dataclass(slots=True)
class ServerStatus:
    """
    Optional heartbeat/status message.
    """

    ready: bool

    backend: str

    model: str

    device: str

    uptime_sec: float

    active_requests: int = 0