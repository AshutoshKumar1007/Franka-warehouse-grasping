"""
Select the best grasp from a set of candidates.

This module contains robot-specific heuristics only.

"""

from __future__ import annotations

from shared.grasp_type import (
    Grasp,
    GraspResult
)

class GraspSelector:
    
    def __init__(self):
        pass
    
    def select(
        self,
        result : GraspResult,
    ) -> Grasp | None:

        if result.empty():
            return None
        
        #
        # Highest score first
        #
        result.sort()
        return result.grasps[0]
    