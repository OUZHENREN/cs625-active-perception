# Frames and topics contract

## 1. Required TF chain

The intended Eye-in-Hand chain is:

```text
world
└── base_link
    └── ...
        └── tool0
            └── camera_mount_link
                └── camera_link
                    ├── camera_color_optical_frame
                    └── camera_depth_optical_frame
```

`target_frame` is published only when a target estimate is available. The camera optical frame must follow the ROS optical-frame convention. There must be one publisher for each transform.

The model follows the senior eye-in-hand layout. In simulation, the Gazebo
sensor keeps the senior `/camera/*` source names. The application bridge maps
them to `/sim/camera/*`; only the common adapter publishes the normalized
`/sensors/camera/*` topics. In real mode, source names remain profile inputs
until the PS800E1/Percipio driver is verified.

| Simulation source | Bridge ROS topic | Adapter output |
|---|---|---|
| `/camera/image` | `/sim/camera/image` | `/sensors/camera/color/image` |
| `/camera/depth_image` | `/sim/camera/depth_image` | `/sensors/camera/depth/image` |
| `/camera/camera_info` | `/sim/camera/camera_info` | `/sensors/camera/depth/camera_info` |
| `/camera/points` | `/sim/camera/points` | `/sensors/camera/points` |

## 2. Normalized application topics

| Topic | Message intent | Publisher boundary |
|---|---|---|
| `/sensors/camera/color/image` | normalized color image | `cs625_sensor_adapter` |
| `/sensors/camera/depth/image` | normalized depth image | `cs625_sensor_adapter` |
| `/sensors/camera/depth/camera_info` | calibrated camera info | `cs625_sensor_adapter` |
| `/sensors/camera/points` | normalized point cloud | `cs625_sensor_adapter` |
| `/sensors/camera/status` | sensor availability/freshness | `cs625_sensor_adapter` |
| `/perception/target_pose` | estimated target pose | later perception layer |
| `/perception/localization_quality` | localization quality | later perception layer |
| `/scene/fused_cloud` | fused scene cloud | later mapping layer |
| `/view_planner/raw_candidates` | reproducible camera candidates in `base_link` | `cs625_view_generation` |
| `/view_planner/reachable_candidates` | candidates that passed TF, IK, collision and planning | `cs625_motion_adapter` |
| `/view_planner/selected_view` | selected candidate | later view evaluation |
| `/motion/status` | plan/execute state and failure code | later motion adapter |
| `/active_localization/state` | task state machine | later orchestrator |

Camera-brand topics must not escape the sensor adapter. Every cloud and pose must include a valid timestamp and `frame_id`. A point cloud must be transformed with `tf2`; it must never be silently treated as `base_link` data.

## 3. Time and freshness contract

- `sim` uses `use_sim_time:=true`;
- `real` uses system time;
- after motion, only a new settled sensor frame is accepted;
- stale point clouds are rejected with an explicit status/failure code;
- target estimates must carry the observation timestamp and source frame.

## 4. Verification commands

Run in the Ubuntu 22.04 Humble VM after the corresponding Phase 1 launch is implemented:

```bash
ros2 topic list
ros2 topic info /sensors/camera/points --verbose
ros2 topic echo /sensors/camera/points --once
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
ros2 run tf2_tools view_frames
```
