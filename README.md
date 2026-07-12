# franka-grasp-assessment

Perception -> grasping -> execution pipeline for the Franka warehouse sim assessment.

This repo contains **only our own packages**. It's meant to sit inside the same
colcon workspace as an *untouched* clone of the provided `franka-warehouse-sim`
repo, side by side under `ros2_ws/src/`. That way a grader can diff the two
cleanly and see exactly what we built vs. what was given.

## Layout

```
franka-grasp-assessment/
├── README.md
├── ros2_ws/
│   └── src/
│       ├── franka_perception/   <- done: sensors -> clean, stable-frame topics
│       ├── franka_grasping/     <- next: point cloud -> grasp poses (GraspNet)
│       └── franka_execution/    <- later: grasp pose -> MoveIt pick-and-place
└── docs/
    └── report.md                <- Part 4 write-up, filled in at the end
```

(`franka-warehouse-sim` clones separately into `ros2_ws/src/`, per the provided
instructions — it is not part of this repo.)

## franka_perception

Subscribes to the raw Gazebo camera bridge topics (`/rgbd_camera/{image,
depth_image, points, camera_info}`, all in the camera's optical frame) and
republishes a stable internal API:

```
/perception/{image, depth, points, camera_info}
```

The point cloud is the only topic carrying 3D geometry, so it's the only one
transformed — from the camera's optical frame into `fr3_link0` (the robot's
base frame), using `tf2_sensor_msgs.do_transform_cloud`. Image/depth/camera_info
are just relayed unchanged, no transform applied.

Camera placement lives in one place, `config/camera.yaml` — the spawn command
and the static TF publisher both read from it, so they can't drift out of sync.

### Build

```bash
cd ros2_ws
colcon build --symlink-install --packages-select franka_perception
source install/setup.bash
```

### Run

With the sim + MoveIt already running (their launch files, in separate terminals):

```bash
ros2 launch franka_perception perception.launch.py world_name:=warehouse_box_large
```

### Verify

```bash
ros2 topic list | grep perception
ros2 topic echo /perception/points --field header.frame_id --once
```

The last command should print `fr3_link0` — confirming the transform happened
and downstream packages never need to think about the camera's raw frame.

## Status

- [x] `franka_perception` — camera bring-up + stable-frame republishing
- [ ] `franka_grasping` — point cloud -> grasp poses (GraspNet/AnyGrasp)
- [ ] `franka_execution` — grasp pose -> MoveIt pick-and-place
