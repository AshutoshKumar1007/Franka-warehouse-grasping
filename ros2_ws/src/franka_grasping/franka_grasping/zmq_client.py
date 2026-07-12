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

import logging
import time
import traceback
from typing import Optional

import zmq

from shared.protocol import (
    InferenceRequest,
    InferenceResponse
)

_logger = logging.getLogger(__name__)


class ZMQInferenceClient:
    """
    The inference server is expected to expose to a ZeroMQ REP socket.

    NOTE on timeouts: this uses a REQ socket, which enforces strict
    one-request-one-reply alternation. On timeout we close and reopen
    the socket (the standard "Lazy Pirate" pattern) rather than trying
    to recv() again on the same socket. This unblocks the CLIENT
    reliably, but it does NOT cancel the request on the server side —
    if the server is a synchronous recv/process/send loop, it will
    keep computing the abandoned request and silently drop the reply
    once it's done. Under sustained server-side latency spikes this
    can cascade (each abandoned request delays the next one), which
    is exactly what a long streak of exact-timeout-interval warnings
    means: not "server is down", but "server is behind, timeout is
    too tight for it to ever catch up".

    `last_latency_ms` is updated after every attempt (success or
    timeout) so callers can log real round-trip time instead of
    only ever seeing failures.
    """

    def __init__(
        self,
        host: str = 'localhost',
        port: int = 5555,
        timeout: float = 5000,
    ):
        self.host = host
        self.port = port
        self.timeout = timeout

        self.context = zmq.Context.instance()

        self.socket = None

        self.last_latency_ms: Optional[float] = None

        self.connect()

    def connect(self):

        if self.socket is not None:
            self.socket.close(linger=0)

        self.socket = self.context.socket(zmq.REQ)

        self.socket.setsockopt(zmq.RCVTIMEO, int(self.timeout))
        self.socket.setsockopt(zmq.SNDTIMEO, int(self.timeout))

        self.socket.connect(
            f"tcp://{self.host}:{self.port}"
        )

    def close(self):

        if self.socket is not None:
            self.socket.close(linger=0)

            self.socket = None

    def infer(
        self,
        request: InferenceRequest,
        max_retries: int = 0,
    ) -> Optional[InferenceResponse]:
        """
        Send one inference request.

        max_retries=0 (default): a single attempt, same behavior as
        before — one timeout drops this point cloud and returns None.

        max_retries>0: RESENDS the same request after a timeout,
        up to max_retries extra times. Only turn this on if you've
        confirmed (via last_latency_ms / the logs) that the server
        is just occasionally slow, not overloaded — retries add MORE
        load onto an already-behind server and can make a backlog
        worse, not better.
        """

        attempt = 0

        while True:
            start = time.monotonic()
            try:
                payload = request.to_bytes()
                self.socket.send(payload)

                reply = self.socket.recv()

                self.last_latency_ms = (time.monotonic() - start) * 1000.0

                return InferenceResponse.from_bytes(reply)

            except zmq.error.Again:
                self.last_latency_ms = (time.monotonic() - start) * 1000.0

                _logger.warning(
                    "Timeout after %.0f ms waiting for inference "
                    "server response (attempt %d/%d).",
                    self.last_latency_ms,
                    attempt + 1,
                    max_retries + 1,
                )

                # Reset the socket to a clean state regardless — a
                # REQ socket that timed out mid-recv cannot be reused
                # for a fresh send() without this.
                self.connect()

                if attempt >= max_retries:
                    return None

                attempt += 1

            except Exception:
                self.last_latency_ms = (time.monotonic() - start) * 1000.0

                _logger.exception("Unexpected ZMQ client error.")

                self.connect()

                return None