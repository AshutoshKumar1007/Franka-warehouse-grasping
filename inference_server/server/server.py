""" 
ZeroMQ inference server

Responsibilities
----------------
- Bind REP socket
- Receive inference requests
- Deserialize request
- Delegate inference to Engine
- Serialize response
- Return result

"""
from __future__ import annotations

import traceback

import zmq

from .engine import InferenceEngine
from shared.protocol import InferenceRequest, InferenceResponse

class InferenceServer:
    
    def __init__(
        self,
        engine : InferenceEngine,
        host : str = '0.0.0.0',
        port : int = 5555,        
    ):
        self.engine = engine
        self.host = host
        self.port = port
        
        self.context = zmq.Context.instance()
        
        self.socket = self.context.socket(zmq.REP)
        
        self.socket.bind(
            f"tcp://{self.host}:{self.port}"
        )
    
    def spin(self):
        print(
            f"[Server] Listening on tcp://{self.host}:{self.port}"
        )
        try:
            while True:
                try:
                    
                    request_bytes = self.socket.recv()
                    
                    request = InferenceRequest.from_bytes(request_bytes)
                    response : InferenceResponse = self.engine.infer(request)
                    
                except Exception as e:
                    traceback.print_exc()
                    
                    response = InferenceResponse(
                        success=False,
                        message=str(e),
                        result=None
                    )
                self.socket.send(response.to_bytes())
        except KeyboardInterrupt:
            print("\nServer interrupted.")
        finally:
            self.close()
    def close(self):
        self.socket.close()
    
