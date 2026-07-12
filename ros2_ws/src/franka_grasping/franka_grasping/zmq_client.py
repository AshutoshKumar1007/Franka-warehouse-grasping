"""
ZeroMQ client used by the ROS grasp node.

Responsibilities
----------------
- Connect to the inference server
- Serialize requests
- Send point clouds
- Receive inference results
- Handle timeouts and reconnection

This module intentionally knows NOTHING about ROS.
"""

from __future__ import annotations

import pickle
import traceback
from typing import Optional

import zmq

from shared.protocol import (
    InferenceRequest,
    InferenceResponse
)

class ZMQInferenceClient:
    """
    The inference server is expected to expose to a ZeroMQ REP socket. 
    """
    def __init__(
        self,
        host : str = 'localhost',
        port : int = 5555,
        timeout : float = 5000
    ):
        self.host = host
        self.port = port
        self.timeout = timeout
        
        self.context = zmq.Context.instance()
        
        self.socket = None
        
        self.connect()
        
    def connect(self):
        
        if self.socket is not None:
            self.socket.close(linger = 0)
        
        self.socket = self.context.socket(zmq.REQ)
        
        self.socket.setsockopt(zmq.RCVTIMEO, int(self.timeout))
        self.socket.setsockopt(zmq.SNDTIMEO, int(self.timeout))
        
        self.socket.connect(
            f"tcp://{self.host}:{self.port}"
        )
    
    def close(self):
        
        if self.socket is not None:
            self.socket.close(linger = 0)
            
            self.socket = None
    def infer(
        self,
        request : InferenceRequest,
    ) -> Optional[InferenceResponse]:

        try:
            payload = request.to_bytes()
            self.socket.send(payload)
            
            reply = self.socket.recv()
            
            response = InferenceResponse.from_bytes(reply)
            
            return response
        except zmq.error.Again:
            print("ZMQInferenceClient: Timeout waiting for server response.")
            self.connect()
            return None
        except Exception as e:
            #
            # reconnect once
            #
            traceback.print_exc()
    
            self.connect()
            
            return None