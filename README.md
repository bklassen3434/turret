# Multi-Sensor Tracking Turret

A ROS 2 project demonstrating a pan-tilt turret that uses sensor fusion (camera + radar) to track multiple drone targets. Built for learning C++ and ROS 2, targeting defense/robotics portfolio applications.

## Features

- **Multi-target tracking** with two simulated drone targets flying independent trajectories
- **Pan-tilt turret model** with camera and simulated radar sensors
- **Computer vision** for multi-target detection (color-based tracking via OpenCV)
- **Simulated radar** (two instances) providing range, range-rate, and bearing measurements
- **Multi-target Kalman filter fusion** with greedy nearest-neighbor data association
- **Track lifecycle management** (tentative, confirmed, coasting, lost)
- **PID control** for smooth turret tracking of the closest confirmed target
- **Drone silhouette rendering** with perspective scaling in the synthetic camera
- **RViz visualization** with drone-shaped markers placed at true 3D track positions, velocity arrows, Kalman covariance ellipsoids, fading trajectory trails, and track labels

## Architecture

```
┌─────────────────────┐
│   Target Simulator  │
│   (2 drone TFs)     │
└──┬──────┬───────┬───┘
   │      │       │
   │      │       │
   ▼      ▼       ▼
┌──────┐ ┌─────┐ ┌─────┐
│Camera│ │Radar│ │Radar│
│ Sim  │ │Sim 1│ │Sim 2│
└──┬───┘ └──┬──┘ └──┬──┘
   │        │       │
   ▼        └───┬───┘
┌──────┐        │
│Vision│        │
│Detect│        │
└──┬───┘        │
   │ N x CameraDetection
   │            │ 2 x RadarDetection
   └─────┬──────┘
         │
   ┌─────▼──────────┐
   │     Fusion      │
   │ (Multi-Target   │
   │  Kalman Filter) │
   └─────┬──────────┘
         │
         │ TrackedTargetArray
         │ (primary_target_id = closest)
         │
   ┌─────▼─────┐
   │  Control   │
   │   (PID)    │
   └─────┬─────┘
         │
   ┌─────▼──────────┐
   │ Joint Simulator │
   └────────────────┘
```

## Packages

| Package | Description |
|---------|-------------|
| `turret_msgs` | Custom message definitions (`CameraDetection`, `RadarDetection`, `TrackedTarget`, `TrackedTargetArray`) |
| `turret_description` | URDF model and RViz config |
| `turret_vision` | Synthetic camera (drone rendering) and vision detector (multi-contour detection) |
| `turret_radar` | Simulated radar sensor with noise modeling (one instance per target) |
| `turret_fusion` | Multi-target Kalman filter with data association and track lifecycle |
| `turret_control` | PID controller and joint simulator |
| `turret_bringup` | Target simulator (dual drone TFs) and launch files |

## Nodes

| Node | Package | Description |
|------|---------|-------------|
| `target_sim_node` | `turret_bringup` | Broadcasts two drone TFs (`target_drone_1::link`, `target_drone_2::link`) in figure-8 patterns at 30Hz |
| `synthetic_camera_node` | `turret_vision` | Looks up both drone TFs, renders drone silhouettes with perspective scaling and noise |
| `vision_detector_node` | `turret_vision` | Detects all red contours in camera image, publishes one `CameraDetection` per contour |
| `radar_sim_node` (x2) | `turret_radar` | Each instance watches one drone TF, publishes `RadarDetection` with noise |
| `fusion_node` | `turret_fusion` | Multi-target tracker with greedy nearest-neighbor association and track lifecycle |
| `target_marker_node` | `turret_fusion` | Publishes drone-shaped RViz markers for confirmed tracks |
| `turret_controller_node` | `turret_control` | PID controller that steers turret toward the primary (closest) target |
| `joint_simulator_node` | `turret_control` | Simulates turret motor response |

## Multi-Target Fusion

The fusion node maintains a map of tracks, each with its own Kalman filter instance (state: `[x, y, z, vx, vy, vz]`).

**Data association** uses greedy nearest-neighbor:
- Camera detections: angular distance threshold (default 0.15 rad)
- Radar detections: combined range + angle distance threshold (default 1.0 m)
- Unassociated detections create new tentative tracks

**Track lifecycle:**
- **Tentative**: New track, needs 3 consecutive updates to confirm
- **Confirmed**: Active track with recent sensor updates
- **Coasting**: No updates for 0.5s, still predicting
- **Lost**: No updates for 2.0s, track is removed

**Primary target selection**: The closest confirmed track by range is designated as the primary target for the turret controller.

## Prerequisites

- Ubuntu 22.04+
- ROS 2 (Humble or Jazzy)
- Required packages:
  ```bash
  # For Humble:
  sudo apt install ros-humble-robot-state-publisher \
                   ros-humble-xacro \
                   ros-humble-cv-bridge \
                   ros-humble-vision-opencv \
                   ros-humble-tf2-ros \
                   ros-humble-tf2-geometry-msgs \
                   libeigen3-dev

  # For Jazzy, replace 'humble' with 'jazzy' above
  ```

## Quick Start

```bash
# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Navigate to workspace and build
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Run the simulation
ros2 launch turret_bringup turret_demo.launch.py
```

## Verification

```bash
# Verify two drone TFs exist
ros2 run tf2_ros tf2_echo base_link target_drone_1::link
ros2 run tf2_ros tf2_echo base_link target_drone_2::link

# Check radar detections from both drones
ros2 topic echo radar/detections

# Check camera detects multiple contours
ros2 topic echo camera/detections

# Check fusion outputs multiple tracks
ros2 topic echo tracks
```

In RViz you should see:
- Two drone-shaped markers positioned at the tracker's true 3D estimate (red = primary/closest, orange = secondary, dim yellow = coasting/predicting)
- A blue velocity arrow on each track showing estimated heading and speed
- A translucent covariance ellipsoid per track that shrinks as camera + radar agree (the Kalman filter converging)
- A fading trajectory trail tracing each drone's flight path (stored in `base_link`, so it stays world-stable as the turret slews)
- Track labels showing ID, `[PRI]` tag, range, contributing sensors (`[CR]`) and confidence
- TF axes for the ground-truth drone frames, so you can compare estimate vs. truth
- The turret smoothly following the closer drone and switching when the other approaches

To run headless (no RViz):

```bash
ros2 launch turret_bringup turret_demo.launch.py use_rviz:=false
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/turret/camera/image_raw` | `sensor_msgs/Image` | Synthetic camera image with drone silhouettes |
| `/turret/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `camera/detections` | `turret_msgs/CameraDetection` | Per-contour bearing detections |
| `camera/debug_image` | `sensor_msgs/Image` | Annotated detection image |
| `radar/detections` | `turret_msgs/RadarDetection` | Range/bearing detections (from both radar instances) |
| `tracks` | `turret_msgs/TrackedTargetArray` | Fused multi-target tracks with primary target ID |
| `/target_markers` | `visualization_msgs/MarkerArray` | Drone-shaped RViz markers |
| `/turret_position_controller/commands` | `std_msgs/Float64MultiArray` | Joint position commands |

## Tuning

### Target simulator
- `speed` / `drone2_speed` - Drone movement speed
- `target_range` / `drone2_range` - Distance from turret
- `drone2_phase_offset` - Azimuth offset between drones
- `num_targets` - Number of drones (1 or 2)

### Vision parameters
- `target_color_lower/upper` - HSV color range for detection
- `min_contour_area` - Minimum contour area to count as a detection
- `max_detections_per_frame` - Cap on detections per frame (default 10)

### Radar parameters
- `range_noise_std` - Range measurement noise (meters)
- `azimuth_noise_std` / `elevation_noise_std` - Bearing noise (radians)
- `detection_probability` - Simulated detection probability

### Fusion parameters
- `association_threshold_camera` - Max angular distance for camera association (radians)
- `association_threshold_radar` - Max distance for radar association (meters)
- `min_hits` - Updates needed to promote tentative to confirmed
- `coast_timeout` - Time before confirmed track starts coasting (seconds)
- `track_timeout` - Time before coasting track is dropped (seconds)

### Control parameters
- `yaw_kp/ki/kd` - Yaw axis PID gains
- `pitch_kp/ki/kd` - Pitch axis PID gains
- `max_velocity` - Maximum joint velocity (rad/s)

### Visualization parameters (`target_marker_node`)
- `fixed_frame` - World-stable frame used for trajectory trails (default `base_link`)
- `trail_length` - Max number of points kept per trail (default 120)
- `trail_min_step` - Minimum move (m) before a new trail point is added (default 0.02)
- `covariance_sigma` - Sigma multiple for the covariance ellipsoid size (default 2.0)

## Frame Conventions

- **base_link / radar_link**: X forward, Y left, Z up
- **camera_optical_frame**: X right, Y down, Z forward
- **Fusion frame**: Positive azimuth = left (matches radar_link Y-axis)

## License

MIT License - see [LICENSE](LICENSE) for details.
