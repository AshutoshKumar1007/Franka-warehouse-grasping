# Franka Warehouse Grasping

A perception → grasping → manipulation pipeline built on top of the provided
`franka-warehouse-sim` assessment environment. A Franka FR3 arm in Gazebo picks
a box off a table using a learned 6-DoF grasp model
([Contact-GraspNet](https://github.com/NVlabs/contact_graspnet), via the
dependency-light [PyTorch port](https://github.com/elchun/contact_graspnet_pytorch))
and places it at a target location on the same table, planned and executed
through MoveIt 2.

This repo contains the packages built for the assessment, layered into the same
workspace as the provided sim packages.

## 1. Pipeline Overview

The grasping pipeline is intentionally organized as a collection of independent modules, each responsible for a single stage of the perception-to-manipulation workflow. Modules communicate exclusively through ROS2 topics (within the robotics stack) or a lightweight ZeroMQ protocol (between ROS and the deep-learning inference backend), allowing individual components to be developed, tested, and replaced independently.

```text
                                Gazebo Simulation
                                       │
                          RGB-D Camera (Image / Depth / Point Cloud)
                                       │
                                       ▼
                            franka_perception
             Camera bridging • TF transformation • Frame normalization
                                       │
                                       ▼
                           franka_preprocessing
         Workspace cropping • Voxel downsampling • Outlier filtering
                                       │
                         /preprocessing/points (PointCloud2)
                                       │
                                       ▼
                             franka_grasping
                  PointCloud2 → NumPy → ZeroMQ Request
                                       │
                           ZeroMQ (REQ / REP Protocol)
                                       │
                                       ▼
                         inference_server (Python)
             Contact-GraspNet • Candidate generation • Ranking
                                       │
                         Canonical grasp representation
                                       │
                                       ▼
                             franka_grasping
              Candidate filtering • Best grasp selection • RViz markers
                                       │
                /grasp_candidate (franka_interfaces::GraspCandidate)
                                       │
                                       ▼
                         franka_manipulation (C++)
           MoveIt planning • Trajectory execution • Gripper control
                                       │
                                       ▼
                              Franka FR3 Execution
```

### Design Philosophy

The system is intentionally divided into loosely coupled components rather than a monolithic ROS node.

- **Perception** is responsible only for acquiring and transforming sensor data into a robot-centric representation.
- **Preprocessing** prepares the point cloud for inference while remaining independent of the grasping algorithm.
- **Grasp Generation** handles communication with the external AI model and remains agnostic to the specific backend implementation.
- **Manipulation** focuses exclusively on motion planning and execution through MoveIt2.

A key architectural decision is the separation between the ROS ecosystem and the deep-learning inference backend using **ZeroMQ**. The inference server runs as an independent Python process with its own virtual environment and dependencies, allowing grasping models to be replaced or upgraded without modifying the ROS pipeline. As long as the communication protocol remains unchanged, alternative grasp generation models (e.g. AnyGrasp, GraspNet, GSNet) can be integrated transparently.

For the current implementation, the grasp node processes a continuous stream of point clouds published by the preprocessing pipeline. To ensure thread-safe execution and prevent overlapping inference requests, access to the inference pipeline is synchronized using a `threading.Lock` rather than a simple boolean flag. This guarantees that only a single inference request is active at any time, even under continuous sensor streaming.

To improve robustness, the communication layer also exposes configurable ROS parameters for server timeout and maximum retry attempts (`server_timeout_ms` and `server_max_retries`), allowing the inference behavior to be tuned without modifying the source code. Each inference request additionally records and logs the end-to-end round-trip latency, providing lightweight runtime monitoring of the communication overhead between the ROS pipeline and the external Contact-GraspNet server.

While this synchronization mechanism is sufficient for a single-camera, single-object pipeline, future iterations could adopt a fully event-driven architecture using ROS Actions, Services, or asynchronous task queues to better support multiple concurrent perception sources and higher-throughput inference.

## 2. Repository Structure

The repository consists of three primary components: the ROS2 workspace, an external inference server, and a small shared communication library.

```text
Vegam/
├── activate.sh
│   Environment activation script used throughout development.
│   Sources ROS2 Humble, the workspace overlays, and required environment variables.
│
├── inference_server/
│   Standalone deep-learning backend executed outside ROS.
│   Owns its own Python virtual environment and communicates
│   with ROS exclusively through ZeroMQ.
│
│   ├── server/
│   │   Server runtime, inference engine, communication protocol,
│   │   and Contact-GraspNet wrapper.
│   │
│   ├── checkpoints/
│   │   Pre-trained Contact-GraspNet model weights.
│   │
│   ├── third_party/
│   │   External Contact-GraspNet implementation and supporting code.
│   │
│   └── docker/
│       Optional containerized deployment.
│
├── shared/
│   Communication layer shared between ROS and the inference server.
│   Defines the request/response protocol together with the canonical
│   grasp representation, ensuring both processes exchange identical data
│   structures without duplicated definitions.
│
└── ros2_ws/
    └── src/
        ├── franka_interfaces/
        │   Custom ROS2 message definitions.
        │
        ├── franka_perception/
        │   Camera deployment, ROS-Gazebo bridges,
        │   frame transformations and preprocessing.
        │
        ├── franka_grasping/
        │   ZeroMQ client, grasp selection,
        │   visualization and ROS integration.
        │
        ├── franka_manipulation/
        │   C++ MoveIt2 wrapper responsible for
        │   motion planning and grasp execution.
        │
        ├── franka_warehouse_world/
        │   Modified simulation world and launch configuration.
        │
        ├── franka_ros2/
        ├── libfranka/
        ├── franka_description/
        └── olvx_descriptions_module/
            Upstream simulation and robot description packages.
```

### 2.1 Project Organization

The repository intentionally separates robotics infrastructure from machine-learning inference.

- The **ROS2 workspace** contains all robot-facing components responsible for sensing, planning, visualization, and manipulation.
- The **inference server** encapsulates all deep-learning dependencies and model execution in an isolated Python environment.
- The **shared** package defines a common communication protocol used by both processes, eliminating duplicated serialization logic and providing a stable interface between ROS and the inference backend.

This organization keeps the robotics stack independent of any particular grasp generation model while allowing the inference backend to evolve without affecting the rest of the system.---

## 3. Getting started

```bash
# 1 — simulation
cd ~/Vegam && source activate.sh
ros2 launch franka_warehouse_world warehouse.launch.py \
    world:=small load_gripper:=true rviz:=false

# 2 — MoveIt
cd ~/Vegam && source activate.sh
ros2 launch franka_warehouse_world moveit.launch.py \
    world:=small load_gripper:=true

# 3 — perception
cd ~/Vegam && source activate.sh
ros2 launch franka_perception perception.launch.py world_name:=warehouse_box_small

# 4 — preprocessing
cd ~/Vegam && source activate.sh
ros2 launch franka_perception preprocessing.launch.py

# 5 — inference server (separate venv, not the ROS workspace)
cd ~/Vegam/inference_server && source venv/bin/activate
python -m server.runner

# 6 — grasping
cd ~/Vegam && source activate.sh
ros2 launch franka_grasping grasp.launch.py

# 7 — manipulation
cd ~/Vegam && source activate.sh
ros2 launch franka_manipulation manipulation.launch.py
```

# 4. Components

---

## 4.1 `franka_perception`

The perception package is responsible for acquiring RGB-D data from the simulation, transforming it into the robot reference frame, and preparing a clean point cloud for grasp generation. It is intentionally divided into two independent nodes so that sensor acquisition and point cloud preprocessing remain loosely coupled.

### Pipeline

```text
Gazebo RGB-D Camera
        │
        ▼
ROS-Gazebo Bridge
        │
        ▼
Perception Node
 • Topic normalization
 • TF transformation
 • Frame conversion
        │
        ▼
/perception/points
        │
        ▼
Preprocessing Node
 • Workspace crop
 • Voxel downsampling
 • Statistical outlier removal (optional)
        │
        ▼
/preprocessing/points
```

### Responsibilities

- Spawn and configure the overhead RGB-D camera.
- Bridge Gazebo sensor topics into ROS2.
- Transform the point cloud into the robot base frame (`fr3_link0`).
- Remove unnecessary geometry before grasp generation.

### Design Notes

Only the point cloud is transformed into the robot frame, since it is the only modality consumed by the downstream grasp generation pipeline. RGB images, depth images and camera calibration are simply republished under a stable internal namespace.

The workspace crop plays a significant role in grasp quality. Removing the table surface, robot links and other irrelevant geometry substantially reduces false grasp candidates and allows the grasp model to focus on the target object.

---

## 4.2 `franka_grasping`

This package forms the bridge between the ROS ecosystem and the AI-based grasp generation backend. It converts incoming point clouds into inference requests, communicates with the external inference server, filters the returned grasp candidates, and publishes the final grasp for execution.

### Pipeline

```text
/preprocessing/points
        │
        ▼
PointCloud2 → NumPy
        │
        ▼
ZeroMQ Client
        │
        ▼
Inference Server
        │
        ▼
Candidate Filtering
        │
        ▼
Grasp Selection
        │
        ├────────► RViz MarkerArray
        │
        ▼
/selected_grasp
```

### Responsibilities

- Convert ROS point clouds into NumPy arrays.
- Send inference requests to the external server.
- Receive and rank grasp candidates.
- Apply robot-specific feasibility constraints.
- Publish RViz visualization markers.
- Publish the selected grasp as `GraspCandidate`.

### Design Notes

Communication with the inference backend is performed through a lightweight ZeroMQ **REQ/REP** socket pair. Both processes share a common serialization layer (`shared/`) containing the communication protocol and canonical grasp representation, allowing the ROS pipeline and inference backend to evolve independently.

Since the preprocessing node continuously streams point clouds, inference requests are synchronized using a thread-safe `threading.Lock`, ensuring that only one request is active at any time. Server timeout, retry count and communication latency are exposed as configurable ROS parameters for runtime tuning.

---

## 4.3 `inference_server`

The inference server hosts the deep-learning grasp generation model outside the ROS workspace. It runs as an independent Python process with its own virtual environment and communicates with ROS exclusively through the ZeroMQ protocol.

### Pipeline

```text
ZMQ Request
      │
      ▼
Runner
      │
      ▼
Inference Engine
      │
      ▼
Contact-GraspNet
      │
      ▼
Canonical Grasp Conversion
      │
      ▼
ZMQ Response
```

### Responsibilities

- Load Contact-GraspNet during startup.
- Maintain a persistent inference service.
- Execute grasp generation on incoming point clouds.
- Convert model outputs into the project's canonical grasp representation.

### Design Notes

Separating inference from ROS keeps the robotics workspace independent of machine-learning dependencies. The server owns its own Python environment and can therefore be updated, containerized, or replaced without rebuilding the ROS workspace.

Internally, the server is organized into three layers:

- **Runner** — manages the ZeroMQ server lifecycle.
- **Inference Engine** — orchestrates the inference pipeline.
- **Model Wrapper** — interfaces directly with Contact-GraspNet.

This abstraction also makes it straightforward to replace Contact-GraspNet with another grasp generation backend while preserving the same communication interface.

---

## 4.4 `franka_manipulation`

The manipulation package is responsible for executing the selected grasp using MoveIt 2. It receives grasp candidates from the grasping node, plans collision-aware trajectories, and executes the complete pick-and-place sequence inside Gazebo.

### Pipeline

```text
/selected_grasp
        │
        ▼
MoveGroupInterface
        │
        ▼
Motion Planning
        │
        ▼
Trajectory Execution
        │
        ▼
Franka FR3
```

### Execution Sequence

```text
Open Gripper
      │
      ▼
Pre-Grasp
      │
      ▼
Cartesian Approach
      │
      ▼
Close Gripper
      │
      ▼
Attach Object
      │
      ▼
Lift
      │
      ▼
Place
      │
      ▼
Detach Object
      │
      ▼
Home
```

### Responsibilities

- Receive grasp candidates from the grasping pipeline.
- Plan collision-aware trajectories using MoveIt 2.
- Execute grasping, lifting and placement motions.
- Maintain the planning scene during manipulation.

### Design Notes

The manipulation pipeline is built around a C++ wrapper over MoveIt 2's `MoveGroupInterface`. Rather than planning directly to the grasp pose, the system first plans to a collision-free pre-grasp pose before executing a short Cartesian approach along the predicted grasp direction. Once the object is grasped, it is attached to the planning scene so that subsequent motion planning correctly accounts for the grasped object.

The current implementation focuses on reliable execution of a single selected grasp. Future work could extend the executor with recovery behaviors, fallback to lower-ranked grasp candidates, and automatic replanning after failed attempts.

---

## 4.5 `franka_interfaces`

A lightweight ROS interface package containing the custom message definitions shared between the Python grasp generation pipeline and the C++ manipulation stack.

### Pipeline

```text
Python Nodes
      │
      ▼
GraspCandidate.msg
      │
      ▼
C++ Nodes
```

### Responsibilities

- Define custom ROS2 message types.
- Provide a language-independent interface between packages.
- Keep communication contracts centralized and versioned.

### Design Notes

The package currently defines the `GraspCandidate` message, which encapsulates the selected grasp pose, gripper width and confidence score. By isolating interfaces into a dedicated package, both Python and C++ components share a single source of truth for message definitions while remaining independently maintainable.

---
## 5. Current Status

The complete perception-to-manipulation pipeline has been implemented and integrated. Sensor acquisition, preprocessing, grasp generation, motion planning, and manipulation communicate through well-defined interfaces and can be developed or replaced independently.

The current system successfully demonstrates:

- End-to-end perception → grasp generation → motion planning → execution.
- Decoupled deep-learning inference through an external ZeroMQ server.
- Modular ROS2 packages with clearly defined responsibilities.
- Collision-aware trajectory planning and execution using MoveIt 2.

The primary limitation at the current stage lies in the quality and consistency of the generated grasp candidates. While the manipulation pipeline reliably executes valid and reachable grasps, the Contact-GraspNet predictions occasionally produce poses that are either physically infeasible or outside the robot's reachable workspace.

Future improvements include:

- More robust grasp candidate ranking and filtering.
- Fallback to lower-ranked candidates after planning failure.
- Event-driven inference instead of continuous point-cloud streaming.
- Support for alternative grasp generation backends while preserving the same communication interface.

---
---
## 6. Modifications to the Provided Simulation

Several changes were made to the original simulation environment to support a complete autonomous grasping pipeline.

### Motion Execution

- Configured the `fr3_arm_controller` to accept and execute trajectories generated by MoveIt 2.
- Integrated the manipulation package with the existing Gazebo simulation for end-to-end execution.

### Perception

- Added an overhead RGB-D camera model.
- Implemented Gazebo-to-ROS topic bridging.
- Added TF integration for transforming sensor data into the robot base frame.

### Test Environment

- Added a dedicated test world containing a smaller graspable box.
- The original warehouse objects exceeded the Franka Hand's maximum opening, preventing successful grasps regardless of grasp quality.

### Pipeline Integration

- Added custom ROS2 packages for perception, preprocessing, grasp generation, manipulation, and message interfaces.
- Introduced a standalone inference server communicating through ZeroMQ, allowing the AI backend to remain independent of the ROS workspace.

------
## 7. Acknowledgments

This project builds upon the following open-source software:

- **Franka Warehouse Simulation** — simulation environment and robot infrastructure provided as part of the technical assessment.
- **franka_ros2**, **libfranka**, **franka_description**, and related Franka packages — robot description, controllers, and MoveIt integration.
- **Contact-GraspNet** (NVIDIA Research) — learning-based grasp generation model.
- **contact_graspnet_pytorch** — PyTorch implementation used as the inference backend.

Special thanks to the authors and maintainers of these open-source projects for making their work publicly available.---

