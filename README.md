# pm01_ws

ROS 2 workspace for the PM01 robot. It uses [FAST-LIO](https://github.com/hku-mars/FAST_LIO) with the robot's Livox Mid-360 LiDAR to build a point-cloud map and to publish the odometry Nav2 needs (`/odom` and TF `odom → base_footprint`).

**Environment:** ROS 2 Jazzy, Ubuntu 24.04. This PC and the robot must use the same `ROS_DOMAIN_ID` (69) and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

## Packages (`src/`)

| Package | Purpose |
|---|---|
| `pm01_mapping/FAST_LIO` | Git submodule: [PiusLim373/FAST_LIO](https://github.com/PiusLim373/FAST_LIO), branch `pm01`. It is built on the `tmms` branch, which fixes FAST-LIO freezing after about 3 minutes of mapping, and adds the PM01 launch defaults. Uses the PM01 config `config/pm01.yaml` and `rviz/pm01.rviz` by default. Publishes `/Odometry`, `/cloud_registered`, `/cloud_registered_body`, `/downsampled_fastlio_map` and TF `camera_init → body`. The previous in-tree copy is in `../archived_pm01_ws_package/FAST_LIO`. |
| `pm01_nav_bridge` | Turns FAST-LIO's pose into Nav2 odometry. `odom_publisher_node.py` publishes `/odom` and TF `odom → base_footprint → LINK_BASE`. See [its README](src/pm01_nav_bridge/README.md) for the frames and the math. |
| `resource` | Installs the PM01 meshes from `src/simulation/mujoco/assets` so the robot's `/robot_description` (mesh paths `package://resource/robot/pm_v2/meshes/…`) displays in RViz on this PC. |
| `pm01_lidar_adapter` | **Not used any more.** It converted the robot's old per-packet `PointCloud2` stream. The robot's Livox driver now publishes `CustomMsg`, which FAST-LIO reads directly. |
| `livox_ros_driver2` | Livox ROS 2 driver ([upstream](https://github.com/Livox-SDK/livox_ros_driver2)). Needed here for the `CustomMsg` message definition. |
| `Livox-SDK2` | Livox SDK2 ([upstream](https://github.com/Livox-SDK/Livox-SDK2)). |

## Data flow

```text
robot: run_livox.sh (Livox driver, xfer_format = 1)
  /livox/lidar (livox_ros_driver2/CustomMsg) ──┐
  /livox/imu ──────────────────────────────────┴─► FAST-LIO (lidar_type: 1) ──► /Odometry (camera_init → body)
                                                                                     │
robot: robot_state_publisher ── TF LINK_BASE → … ──► odom_publisher_node.py ◄────────┘
                                                        └─► /odom, TF odom → base_footprint → LINK_BASE,
                                                            static TF odom → camera_init
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd pm01_ws
git submodule update --init --recursive   # first time: fetches FAST_LIO and its ikd-Tree
colcon build --symlink-install
```

Build with no conda environment active (`conda deactivate`). CMake prefers the conda env's Python whenever `CONDA_PREFIX` is set, and FAST-LIO's message generation then fails.

If `livox_ros_driver2/package.xml` is missing, copy `package_ROS2.xml` to `package.xml` in that folder.

## Run

First, on the robot: the supervisor service must be running `run_livox.sh` (the Livox driver and `robot_state_publisher`).

Then open two terminals on this PC. In each one, run these two lines first:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**Terminal 1: FAST-LIO**

```bash
ros2 launch fast_lio mapping.launch.py
```

The robot's Livox driver must publish whole 10 Hz frames. Check that `ros2 topic hz /livox/lidar` shows about 10 Hz, not about 2000 Hz; per-packet messages break FAST-LIO.

**Terminal 2: odometry for Nav2**

```bash
ros2 launch pm01_nav_bridge odom_publisher.launch.py
```

Keep the robot **standing still with both feet on the floor** until terminal 2 logs `odom initialised` (about 1 s). Restart terminal 2 whenever FAST-LIO restarts. Don't run the old `nav_bridge.launch.py` or `lidar_adapter` at the same time.

RViz opens with FAST-LIO. Set its fixed frame to `odom` to see the map levelled, with the floor at z ≈ 0.

## Topics and frames

| Topic | Type | Frame | Notes |
|---|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | `camera_init → body` | Raw FAST-LIO pose of the LiDAR's IMU |
| `/odom` | `nav_msgs/Odometry` | `odom → base_footprint` | Flat robot pose and velocity, for Nav2 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | `body` | Current scan, motion-corrected; the intended input for Nav2's costmap |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | `camera_init` | Current scan in the world frame |
| `/downsampled_fastlio_map` | `sensor_msgs/PointCloud2` | `camera_init` | Voxel-downsampled map for viewing. It is latched (transient local) and resent every 5–30 s. Don't feed it to Nav2. |

## Status and known issues

- `/odom` has been checked with the robot standing still (2026-11-02), but not yet while walking. Don't use it for autonomous locomotion until the walking checks in the [`pm01_nav_bridge` README](src/pm01_nav_bridge/README.md#checking-it-works) pass.
- FAST-LIO drifts by about 1 cm per minute while the robot stands still.
- The robot's clock is about 7.6 s ahead of this PC's. The odometry is unaffected, but Nav2 needs the clocks synced (for example with chrony).
