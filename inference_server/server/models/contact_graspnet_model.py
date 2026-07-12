"""
Thin wrapper around Contact-GraspNet.

Responsibilities
----------------
1. Load Contact-GraspNet configuration.
2. Load model checkpoint.
3. Own a single GraspEstimator instance.
4. Convert repository output into our canonical grasp format.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Dict, Any, Optional

import numpy as np

from contact_graspnet_pytorch.config_utils import load_config
from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch.checkpoints import CheckpointIO
class ContactGraspNetModel:
    """ 
    Thin wrapper around Contact-GraspNet.
    Input
    -----
    point_cloud : numpy.ndarray (N,3)

    Output
    ------
    list[dict]
    """
    def __init__(
        self,
        checkpoint_dir: str,
    ):
        
        self.checkpoint_dir = Path(checkpoint_dir)
        
        self.config = None
        self.estimator = None
        self.checkpoint_io = None
        
    
    def load(self):

        print("[ContactGraspNet] Loading configuration...")

        self.config = load_config(
            str(self.checkpoint_dir),
            batch_size=1,
            arg_configs=[],
        )

        print("[ContactGraspNet] Loading model checkpoint...")

        self.estimator = GraspEstimator(
            self.config
        )

        model_checkpoint_dir = (
            self.checkpoint_dir / "checkpoints"
        )

        self.checkpoint_io = CheckpointIO(
            checkpoint_dir=str(model_checkpoint_dir),
            model=self.estimator.model,
        )

        print("[ContactGraspNet] Loading weights...")
        
        load_dict = self.checkpoint_io.load("model.pt")

        # print(load_dict)
        print("[ContactGraspNet] Ready.") 
        
        
    def infer(
        self,
        point_cloud: np.ndarray,
        segmentation: Optional[np.ndarray] = None,
    ):
        if self.estimator is None:
            raise RuntimeError("Model not loaded. Call `load()` before inference.")
        
        pred_grasps_cam, scores, contact_pts, gripper_openings = (
            self.estimator.predict_scene_grasps(
                pc_full=point_cloud,
                pc_segments={}, #No segmentation 
                local_regions= False,
                filter_grasps= False,
                forward_passes=1,
            )
        )
        # print(type(pred_grasps_cam))
        # print(type(scores))
        # print(type(gripper_openings))

        # print(pred_grasps_cam)
        
        return self._convert_output(
            pred_grasps_cam,
            scores,
            gripper_openings,
        )
        
    def _convert_output(
        self,
        pred_grasps_cam,
        scores,
        gripper_openings,
    ):
        
        results = []
        
        for object_id in pred_grasps_cam:
            grasps = pred_grasps_cam[object_id]
            grasp_scores = scores[object_id]
            grasp_widths = gripper_openings[object_id]
            
            grasp_widths = np.atleast_1d(grasp_widths)
            for grasp, score, width in zip(
                grasps,
                grasp_scores,
                grasp_widths,
            ):
                results.append(
                    {
                        "score": float(score),

                        "position": grasp[:3, 3].tolist(),

                        "rotation": grasp[:3, :3].tolist(),

                        "width": float(width),
                    }
                )
        #
        # Highest confidence first.
        #
        results.sort(
            key=lambda g: g["score"],
            reverse=True,
        )
        
        return results
