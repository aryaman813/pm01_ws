# pm01_nav_bridge

This package turns FAST-LIO's LiDAR pose into the odometry and frames that Nav2 needs on the PM01 humanoid.

| Executable | Status |
|---|---|
| `odom_publisher_node.py` | **Current.** Everything below describes this node. |
| `nav_bridge` (C++) | Legacy. Kept only for comparison; see [Legacy C++ bridge](#legacy-c-bridge). **Never run both at once**: both publish `/odom` and both claim a parent for `LINK_BASE`. |

## Quick start

```bash
ros2 launch fast_lio mapping.launch.py
ros2 launch pm01_nav_bridge odom_publisher.launch.py
```

- After both start, the robot must **stand still with both feet on the floor for about 1 s**. The node uses that second to define `odom`. FAST-LIO also needs it for its own IMU initialisation.
- **Restart this node whenever FAST-LIO restarts.** A restarted FAST-LIO creates a new `camera_init`, and the node only computes `odom → camera_init` once.

## What it publishes

| Output | Type | Rate | Contents |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | 10 Hz (FAST-LIO rate) | Header frame `odom`, child frame `base_footprint`. Flat pose (x, y, yaw) and velocity (vx, vy, wz) in `base_footprint`. |
| TF `odom → base_footprint` | dynamic | 10 Hz | x, y, yaw only; z, roll and pitch are always 0 |
| TF `base_footprint → LINK_BASE` | dynamic | 10 Hz | Pelvis height, roll and pitch |
| TF `odom → camera_init` | static | once | Computed at startup |

All timestamps are copied from FAST-LIO's messages, which use the robot's clock. The node never reads the PC's clock, so a clock offset between the robot and the PC doesn't affect it. Nav2 itself still needs the clocks synced.

## TF tree

```text
odom ─► base_footprint ─► LINK_BASE ─► legs, LINK_TORSO_YAW ─► livox_frame   (robot_state_publisher on the robot)
  │      x, y, yaw only     z, roll, pitch
  └─► camera_init ─► body                                                    (FAST-LIO)
      static, set at startup
```

On a wheeled robot, `base_footprint → base_link` is a fixed offset. On a humanoid, the pelvis rises, falls, tilts and sways with every step. So this node recalculates that link on every update, and Nav2 only ever sees a flat `base_footprint` moving over a flat `odom`.

## Frames

| Frame | Defined by | Fixed to | Relation to the URDF |
|---|---|---|---|
| `odom` | This node | **World.** The floor directly under the pelvis at startup; z up (against gravity); x = the pelvis's forward direction at startup. | Not in the URDF |
| `camera_init` | FAST-LIO | **World.** A snapshot of where `body` was when FAST-LIO started. The name is left over from LOAM; no camera is involved. Because the LiDAR is mounted sideways, this frame is sideways too. | Not in the URDF |
| `body` | FAST-LIO | Robot: the Mid-360's internal IMU | `livox_frame` shifted by (0.011, 0.023, −0.044) m, same axes |
| `livox_frame` | URDF | Robot: the LiDAR. X up, Y right, Z forward. | Fixed child of `LINK_TORSO_YAW` |
| `LINK_TORSO_YAW` | URDF | Robot: the torso, after the waist yaw joint (`J12_WAIST_YAW`) | Its own link |
| `LINK_BASE` | URDF (root) | Robot: the pelvis | Its own link |
| `base_footprint` | This node | Robot, flattened: the pelvis projected onto the floor; heading = pelvis heading | Not in the URDF |
| `LINK_FOOT_L` / `LINK_FOOT_R` | URDF | Robot: the soles, fixed 4.63 cm below each ankle roll joint. That's exactly the lowest point of the foot mesh. | Their own links |

`odom` and `camera_init` are both fixed to the world. Every other frame moves with the robot.

## Notation

`T_A_B` is a 4×4 matrix `[R t; 0 0 0 1]`, the same thing as the TF `A → B` (A is the parent, B the child):

- **R** is B's axes expressed in A.
- **t** is the position of B's origin, in A's coordinates.
- **Chaining:** `T_A_C = T_A_B · T_B_C`. The inner letters must match.
- **Reversing:** `T_B_A = T_A_B⁻¹`.

| Short name | Frame |
|---|---|
| `cam` | `camera_init` |
| `body` | FAST-LIO's `body` (the IMU) |
| `livox` | `livox_frame` |
| `base` | **`LINK_BASE`, the pelvis** (never `base_footprint`) |
| `fp` | `base_footprint` |
| `foot` | `LINK_FOOT_L` / `LINK_FOOT_R` |

## Transforms

The node finds the pelvis by following this chain:

```text
odom ──T_odom_cam──► camera_init ──T_cam_body──► body ──T_body_livox──► livox_frame ──T_livox_base──► LINK_BASE
     stage 1, fixed       FAST-LIO, 10 Hz          pm01.yaml, fixed       robot TF, reversed
```

**Inputs (read by the node):**

| Matrix | TF | Source | Changes |
|---|---|---|---|
| `T_cam_body` | `camera_init → body` | FAST-LIO `/Odometry` pose | 10 Hz |
| `T_base_livox` | `LINK_BASE → livox_frame` | Robot TF, looked up at the FAST-LIO timestamp. It combines the waist joint (`LINK_BASE → LINK_TORSO_YAW`) with the fixed URDF mount (`LINK_TORSO_YAW → livox_frame`), then applies `lidar_mount_rpy_offset`. | When the waist turns |
| `T_livox_body` | `livox_frame → body` | `pm01.yaml`: R = `extrinsic_R` = identity, t = −`extrinsic_T` | Never |
| `T_base_foot` | `LINK_BASE → LINK_FOOT_L/R` | Robot TF (leg joints) | Stage 1 only |
| `a_body` (vector) | Gravity in `body` | Average `/livox/imu` accelerometer reading while standing still | Stage 1 only |

**Calculated on every FAST-LIO message (both stages):**

| Matrix | Formula | Meaning |
|---|---|---|
| `T_base_body` | `T_base_livox · T_livox_body` | Pelvis → IMU |
| `T_cam_base` | `T_cam_body · T_base_body⁻¹` | The pelvis pose in FAST-LIO's world |

**Calculated by the stages:**

| Matrix | Formula | Published |
|---|---|---|
| `T_odom_cam` | See stage 1 | Static TF, once |
| `T_odom_base` | `T_odom_cam · T_cam_base` | No (intermediate) |
| `T_odom_fp` | Translation (x, y, 0) from `T_odom_base`; rotation `Rz(yaw)` with `yaw = atan2(R[1,0], R[0,0])` of `T_odom_base` | TF and `/odom` |
| `T_fp_base` | `T_odom_fp⁻¹ · T_odom_base`, which always works out to t = (0, 0, height) with roll and pitch only | TF |

`T_odom_fp · T_fp_base = T_odom_base`, so the published tree is consistent with the full pelvis pose.

## Stage 1: startup (once)

Goal: compute `T_odom_cam`, which places `odom` relative to FAST-LIO's world.

1. **Wait until the robot is still.** Collect `/livox/imu` samples for `init_duration` (1 s). If the gyro magnitude exceeds `gyro_still_threshold`, the robot is moving, so discard the samples and start again.
2. **Find "up" in `camera_init`:** `u_cam = normalize(R_cam_body · a_body)`. At rest, the accelerometer measures the reaction to gravity, which points up.
3. **Level:** `R_level` is the smallest rotation that turns `u_cam` into (0, 0, 1).
4. **Heading:** `yaw0` is the pelvis heading in the levelled frame, `atan2(R[1,0], R[0,0])` of `R_level · R_cam_base`. Then `R_odom_cam = Rz(−yaw0) · R_level`, so the robot starts facing +x.
5. **Floor:** express both feet in the `odom` orientation, `R_odom_cam · (T_cam_base · t_base_foot)`, and take the lower one: `z_floor = min(z_L, z_R) − sole_offset`.
6. **Origin:** with the pelvis position `p = R_odom_cam · t_cam_base`, set `t_odom_cam = −(p.x, p.y, z_floor)`. This puts the origin on the floor directly under the pelvis.
7. Publish `T_odom_cam` as a static TF, log it, and stop reading the IMU.

## Stage 2: every FAST-LIO update

1. Read `T_cam_body` from `/Odometry`. Look up `T_base_livox` at the same timestamp, so a waist turn is matched to the correct scan.
2. `T_cam_base = T_cam_body · (T_base_livox · T_livox_body)⁻¹`
3. `T_odom_base = T_odom_cam · T_cam_base`
4. Split it: `T_odom_fp` gets x, y and yaw; `T_fp_base` gets the rest (height, roll, pitch).
5. Velocity: finite difference of (x, y, yaw) between consecutive messages, using their timestamps. Rotate into `base_footprint` and smooth with an exponential moving average (`velocity_alpha`).
6. Publish both TF links and `/odom`.

The yaw comes from the **pelvis**, not the torso. The waist turns the torso and the LiDAR, but the legs walk where the pelvis points. Using `T_base_livox` at the scan's timestamp removes the waist angle.

## Why the design holds up

- **Squatting or bending after startup doesn't move `odom`.** `odom` and `camera_init` are both fixed to the world. Robot motion only changes `camera_init → body`, which the node then splits:
  - A squat lowers `LINK_BASE` in `base_footprint → LINK_BASE`; `base_footprint` stays on the floor.
  - PM01's waist only twists, so bending forward happens at the hips (`J00`/`J06_HIP_PITCH`) and tilts the pelvis. That pitch goes into `base_footprint → LINK_BASE`. The heading stays correct unless the pelvis tips close to 90°.
- **Any startup posture works.** Level comes from gravity, the floor from the feet, and the heading from the pelvis. The robot only needs to be still with both feet down.
- **`extrinsic_R` = identity is correct even though the LiDAR is sideways.** FAST-LIO's extrinsics describe the LiDAR relative to the IMU *inside* the Mid-360 housing, and their axes are parallel. The sideways mount only appears in the URDF (`LINK_TORSO_YAW → livox_frame`) and in `odom → camera_init`.

## Reference values

Logged by the node on 2026-11-02 with the robot standing upright (`lidar_mount_rpy_offset` = 0):

| Quantity | Value |
|---|---|
| `/livox/imu` at rest | (0.994, 0.001, 0.044) g: gravity along LiDAR +X, tilted 2.5° |
| `odom → camera_init` | t = (0.080, −0.032, 1.120) m, quaternion xyzw = (0.690, 0.000, 0.724, 0.001) |
| Pelvis (`LINK_BASE`) in `odom` | height 0.821 m, roll −0.07°, pitch −2.51° |
| Feet in `odom` | z = 0.000 |

The `odom → camera_init` rotation is almost exactly the URDF's LiDAR mount rotation, (0.707, 0, 0.707, 0), which puts X up and Z forward. The small difference is the measured 2.5° tilt. The translation means the IMU sits 8 cm ahead of the pelvis and 1.12 m above the floor.

**LiDAR mount angle:** with `odom` levelled by gravity, feet standing flat should read 0° pitch. With the URDF mount as-is, they read −2.2° to −3.0°, which means the real LiDAR mount is about 2.5° off from the URDF. A live test with `lidar_mount_rpy_offset: [0.0, 0.0436, 0.0]` (2.5° about `livox_frame` Y) gave:

| | Offset 0 | Offset 0.0436 rad |
|---|---|---|
| Feet pitch (L / R) | −2.2° to −3.0° | −0.4° / −0.8° |
| Pelvis pitch | −2.5° | +0.04° |

This doesn't affect levelling or point-cloud placement, because those use the Livox IMU directly. It only affects the pelvis and leg frames, which are derived through the mount. For comparison, the robot's own IMU (`/hardware/imu_info`) reports pelvis pitch +1.8°, which may have its own mounting offset.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `odom_frame` | `odom` | |
| `footprint_frame` | `base_footprint` | |
| `base_frame` | `LINK_BASE` | Pelvis, the URDF root |
| `lidar_frame` | `livox_frame` | |
| `foot_frames` | `[LINK_FOOT_L, LINK_FOOT_R]` | Used to find the floor at startup |
| `fastlio_odom_topic` | `/Odometry` | |
| `imu_topic` | `/livox/imu` | |
| `lidar_in_imu` | `[-0.011, -0.02329, 0.04412]` | FAST-LIO `extrinsic_T`; keep it equal to `pm01.yaml` |
| `lidar_mount_rpy_offset` | `[0, 0, 0]` | Correction (rad) to the URDF LiDAR mount, about `livox_frame`'s own axes |
| `sole_offset` | `0.0` | Distance (m) from the foot frames down to the floor, if they aren't at the sole |
| `init_duration` | `1.0` | Seconds of stillness needed at startup |
| `gyro_still_threshold` | `0.05` | rad/s; above this, the robot counts as moving |
| `velocity_alpha` | `0.5` | Velocity smoothing; 1.0 means no smoothing |
| `tf_timeout` | `0.05` | Seconds to wait for the waist TF at each scan's timestamp |

## Checking it works

With the robot standing still after startup:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint   # ≈ (0, 0, 0), yaw 0
ros2 run tf2_ros tf2_echo odom LINK_BASE        # z ≈ 0.82
ros2 run tf2_ros tf2_echo odom LINK_FOOT_L      # z ≈ 0
ros2 topic hz /odom                             # ≈ 10 Hz
```

- In RViz with fixed frame `odom`, the floor in `/cloud_registered` should lie flat at z ≈ 0.
- Turning 360° in place should keep `base_footprint` x and y within a few cm. A circle means the mount or IMU offset is wrong.
- Walking 2 m straight should give `/odom` x ≈ 2 m, with the foot frames staying at z ≈ 0. A slope means the levelling is wrong.
- Turning only the waist shouldn't change the `base_footprint` yaw.

## Known limitations

- The covariances are placeholders, not calibrated values.
- The pelvis sways a few cm side to side with each step, so `base_footprint` wobbles with it. Smoothing `odom → base_footprint`, or using the midpoint between the feet, would remove this.
- `base_footprint` is fixed at z = 0, which assumes a flat floor.
- There's no `map` frame yet; Nav2 should use `odom` as its global frame for now.
- A FAST-LIO restart isn't detected automatically. Restart this node with it.
- **FAST-LIO noise passes straight through.** Standing still, the waist joint and both IMUs showed no real motion, yet the pelvis pose jittered. With `filter_size_surf` and `filter_size_map` at 0.5: about 2 mm std, steps up to 7 mm between updates, and 0.12° std in pitch. At 0.2 it dropped to about 1 mm std, 3.6 mm steps and 0.04°, while FAST-LIO's CPU use went from 20% to 26% (2026-11-02, 30 s each).
- **FAST-LIO drift passes straight through.** With the robot standing still, `/Odometry` drifted about 1 cm per minute (2026-11-02). Because `base_footprint` is fixed at z = 0, height drift makes the floor in the point cloud slowly rise or sink relative to `odom`.
- **The floor in the point cloud isn't exactly at z = 0.** Right after startup, the floor within 3 m of the robot was about 3.5 cm above the soles, and from 3 to 5 m within 1 cm of them. The foot frames are confirmed to be at the sole: the foot mesh's lowest point is exactly at `LINK_FOOT_*`. Allow for this when setting Nav2's obstacle height threshold.

## Legacy C++ bridge

`src/nav_bridge.cpp` with `launch/nav_bridge.launch.py` is the earlier version. Its problems:

- It publishes `odom → LINK_BASE` with no `base_footprint`, and the velocity is always zero.
- It levels with a hard-coded −90° pitch (`odom → camera_init` = (0, 0, 0), quaternion (0, −0.707, 0, 0.707)). That ignores the measured 2.5° tilt, puts the floor at z ≈ −1.13 m, and has the robot start facing −x.
- The launch file overrides the LiDAR mount to (0, 0, 0.2191) instead of the URDF's (0.1244, 0, 0.2129), so the pelvis position is about 12 cm off.
- It ignores the IMU-to-LiDAR offset (`extrinsic_T`).
