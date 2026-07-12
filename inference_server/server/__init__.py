#                 runner.py
#                      │
#                      ▼
#              InferenceServer
#                      │
#                      ▼
#              InferenceEngine
#           ┌──────────┼──────────┐
#           ▼          ▼          ▼
#   Preprocessor    Model    Postprocessor
#                      │
#                      ▼
#             InferenceResponse
#                      │
#                      ▼
#              InferenceServer
#                      │
#                      ▼
#                  ZMQ REP