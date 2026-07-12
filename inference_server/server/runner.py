"""
Inference server entry point.

Responsibilities
----------------
1. Create inference engine.
2. Start ZeroMQ server.
"""

from __future__ import annotations

from .engine import InferenceEngine
from .server import InferenceServer


def main():

    engine = InferenceEngine(
        checkpoint_dir=(
            "third_party/contact_graspnet/"
            "checkpoints/contact_graspnet"
        )
    )

    server = InferenceServer(
        engine=engine,
        host="0.0.0.0",
        port=5555,
    )

    server.spin()


if __name__ == "__main__":
    main()