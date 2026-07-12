"""
Inference engine.

Responsibilities
----------------
1. Own a single ContactGraspNetModel instance.
2. Run inference.
3. Return canonical grasp results.

The engine does not know about ZMQ, ROS or serialization.
"""

from .models.contact_graspnet_model import ContactGraspNetModel

from shared.protocol import InferenceRequest, InferenceResponse
from shared.grasp_type import GraspResult, Grasp
class InferenceEngine:
    
    def __init__(self, checkpoint_dir: str):

        self.model = ContactGraspNetModel(
            checkpoint_dir,
        )

        self.model.load()
    
    def infer(
        self,
        request : InferenceRequest,
    ) -> InferenceResponse:
        grasps = self.model.infer(
            point_cloud=request.point_cloud,
            segmentation=request.segmentation,
        )
        result = GraspResult.from_list(
            grasps[:request.max_grasps]
        )


        return InferenceResponse(
            success=True,
            message="ok",
            result=result
        )
































# """ 
# Inference Engine

# Coordinates the complete grasp generation pipeline.

# Pipeline: 
#     InferenceRequest -> Preprocessor -> GraspModel -> Postprocessor -> InferenceResponse
# """

# from __future__ import annotations

# import time 

# from .protocol import InferenceRequest, InferenceResponse
# from .preprocess import GraspPreprocessor
# from .postprocess import GraspPostprocessor

# from .models import GraspModel

# class InferenceEngine:
    
#     def __init__(self, config):
#         self.config = config
        
#         #Pipeline components
#         self.preprocessor = GraspPreprocessor(config)
        
#         self.model = GraspModel(config)
        
#         self.postprocessor = GraspPostprocessor(config)
        
#         # Load model 
#         self.model.load()
        
#     def infer(
#         self,
#         request : InferenceRequest,
#     ) -> InferenceResponse:
#         """
#         Run inference on a point cloud.
#         """
#         t0 = time.perf_counter()
        
#         network_input = self.preprocessor.preprocess(
#             request.point_cloud
#         )
        
#         raw_output = self.model.infer(
#             network_input
#         )
        
#         result = self.postprocessor.postprocess(
#             raw_output,
#             max_grasps=request.max_grasps,
#             score_threshold=request.score_threshold
#         )
        
#         elapsed = (time.perf_counter() - t0)*1000.0
                
#         return InferenceResponse(
#             success=True,
#             message="ok",
#             result=result,
            
#             inference_time=elapsed,
#             backend=self.model.backend
#         )
#     def shutdown(self):
#         self.model.unload()