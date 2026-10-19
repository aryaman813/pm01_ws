# pm01_ws

ROS 2 workspace for the PM01 robot. It uses [FAST-LIO](https://github.com/hku-mars/FAST_LIO) with a Livox Mid-360S LiDAR to build a `PointCloud2` map (`/Laser_map`) and publish odometry (`/odom`).

**Environment:** ROS 2 Jazzy, Ubuntu 24.04

## Packages (`src/`)

| Package | Purpose |
|---|---|
| `pm01_lidar_adapter` | Converts the PM01 LiDAR `PointCloud2` on `/livox/lidar` into the format FAST-LIO expects on `/livox/lidar_modified`. |
| `FAST_LIO` | FAST-LIO with PM01 changes (`config/pm01.yaml`, `src/laserMapping_pm01.cpp`, `rviz/pm01_nav*.rviz`). Publishes `/Laser_map`, `/Odometry` and TF `camera_init -> body`. |
| `pm01_nav_bridge` | Converts FAST-LIO's pose (`camera_init -> body`) into the robot base pose. Publishes `/odom` and TF `odom -> LINK_BASE`. |
| `livox_ros_driver2` | Livox ROS 2 driver ([upstream](https://github.com/Livox-SDK/livox_ros_driver2)). |
| `Livox-SDK2` | Livox SDK2 ([upstream](https://github.com/Livox-SDK/Livox-SDK2)). |

## Data flow

```text
/livox/lidar ──► pm01_lidar_adapter ──► /livox/lidar_modified ──┐
                                                                ├─► FAST-LIO ──► /Laser_map (PointCloud2, frame camera_init)
/livox/imu ─────────────────────────────────────────────────────┘             └─► /Odometry (camera_init -> body)
                                                                                      │
                                              pm01_nav_bridge  ◄──────────────────────┘
                                              (+ waist TF LINK_BASE -> LINK_TORSO_YAW)
                                                      └─► /odom and TF odom -> LINK_BASE
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd pm01_ws
colcon build
```

If `livox_ros_driver2/package.xml` is missing, copy `package_ROS2.xml` to `package.xml` in that folder.

## Run

Open three terminals. In each one, run these two lines first:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**Terminal 1: LiDAR adapter**

```bash
ros2 run pm01_lidar_adapter lidar_adapter
```

**Terminal 2: navigation bridge** (starts the static TF `odom -> camera_init` and the bridge node)

```bash
ros2 launch pm01_nav_bridge nav_bridge.launch.py
```

**Terminal 3: FAST-LIO**

```bash
ros2 launch fast_lio mapping.launch.py config_file:=pm01.yaml
```

RViz opens with FAST-LIO using `rviz/pm01_nav.rviz` by default (Fixed Frame `camera_init`). `rviz/pm01_nav_2.rviz` uses Fixed Frame `odom`.

## Topics and frames

| Topic | Type | Frame | Notes |
|---|---|---|---|
| `/Laser_map` | `sensor_msgs/PointCloud2` | `camera_init` | Accumulated map |
| `/Odometry` | `nav_msgs/Odometry` | `camera_init -> body` | Raw FAST-LIO pose |
| `/odom` | `nav_msgs/Odometry` | `odom -> LINK_BASE` | Pose from `pm01_nav_bridge` |

Notes on `/odom`:

- Only the pose is meaningful. FAST-LIO does not output a velocity estimate, so the twist covariance is set very high.
- The pose covariances are placeholders, not calibrated values.
- The bridge needs a timestamp-matched TF `LINK_BASE -> LINK_TORSO_YAW` from the robot.

## Tuning the alignment

These settings are in `src/pm01_nav_bridge`:

- **`odom -> camera_init` rotation:** set in `launch/nav_bridge.launch.py` (static TF `--roll/--pitch/--yaw`) and `q_world` in `src/nav_bridge.cpp`. Keep the two the same. This rotates how the map is placed relative to `odom` in RViz.
- **LiDAR mount:** `mount_x`, `mount_y` and `mount_z` (metres, relative to `LINK_TORSO_YAW`) in the launch file, and `q_mount` in `src/nav_bridge.cpp`. These change the computed robot pose, not the point cloud.

## Warning

The bridge is for pose and TF validation only. Do not use `/odom` for autonomous locomotion yet.
