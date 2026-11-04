#!/usr/bin/env python3
"""PM01 odometry for Nav2, built from FAST-LIO's LiDAR pose.

Publishes:
    odom -> camera_init       static, computed once at startup
    odom -> base_footprint    x, y, yaw of the pelvis, on the floor
    base_footprint -> LINK_BASE   pelvis height, roll, pitch
    /odom                     same pose as odom -> base_footprint, plus velocity

Notation: T_a_b is the 4x4 transform of frame b expressed in frame a
(the TF "a -> b"). cam = camera_init, body = FAST-LIO IMU frame,
livox = livox_frame, base = LINK_BASE (pelvis), fp = base_footprint.

See README.md in this package for the frame definitions and the math.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)


# ----------------------------------------------------------------------
# Transform helpers
# ----------------------------------------------------------------------

def make_transform(rotation, translation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def transform_from_msg(translation, quaternion):
    rotation = Rotation.from_quat(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_matrix()
    return make_transform(
        rotation, [translation.x, translation.y, translation.z])


def invert(transform):
    rotation_t = transform[:3, :3].T
    return make_transform(rotation_t, -rotation_t @ transform[:3, 3])


def rot_z(yaw):
    return Rotation.from_euler('z', yaw).as_matrix()


def heading(rotation):
    # Direction of the frame's x axis projected onto the floor.
    return math.atan2(rotation[1, 0], rotation[0, 0])


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotation_aligning(u, v):
    """Smallest rotation that turns unit vector u into unit vector v."""
    axis = np.cross(u, v)
    sin_angle = np.linalg.norm(axis)
    cos_angle = float(np.dot(u, v))

    if sin_angle < 1e-9:
        if cos_angle > 0.0:
            return np.eye(3)
        # Opposite vectors: turn 180 degrees about any perpendicular axis.
        perpendicular = np.cross(u, [1.0, 0.0, 0.0])
        if np.linalg.norm(perpendicular) < 1e-6:
            perpendicular = np.cross(u, [0.0, 1.0, 0.0])
        perpendicular /= np.linalg.norm(perpendicular)
        return Rotation.from_rotvec(math.pi * perpendicular).as_matrix()

    angle = math.atan2(sin_angle, cos_angle)
    return Rotation.from_rotvec(axis / sin_angle * angle).as_matrix()


def to_transform_stamped(transform, stamp, parent, child):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child

    msg.transform.translation.x = float(transform[0, 3])
    msg.transform.translation.y = float(transform[1, 3])
    msg.transform.translation.z = float(transform[2, 3])

    x, y, z, w = Rotation.from_matrix(transform[:3, :3]).as_quat()
    msg.transform.rotation.x = float(x)
    msg.transform.rotation.y = float(y)
    msg.transform.rotation.z = float(z)
    msg.transform.rotation.w = float(w)
    return msg


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# ----------------------------------------------------------------------
# Node
# ----------------------------------------------------------------------

class OdomPublisher(Node):

    def __init__(self):
        super().__init__('pm01_odom_publisher')

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.footprint_frame = self.declare_parameter(
            'footprint_frame', 'base_footprint').value
        self.base_frame = self.declare_parameter('base_frame', 'LINK_BASE').value
        self.lidar_frame = self.declare_parameter('lidar_frame', 'livox_frame').value
        self.foot_frames = list(self.declare_parameter(
            'foot_frames', ['LINK_FOOT_L', 'LINK_FOOT_R']).value)

        fastlio_odom_topic = self.declare_parameter(
            'fastlio_odom_topic', '/Odometry').value
        imu_topic = self.declare_parameter('imu_topic', '/livox/imu').value

        # FAST-LIO extrinsic_T: LiDAR origin in the IMU (body) frame.
        # extrinsic_R is identity: inside the Mid-360 the IMU and LiDAR
        # axes are parallel, so body has the same axes as livox_frame.
        lidar_in_imu = np.array(self.declare_parameter(
            'lidar_in_imu', [-0.011, -0.02329, 0.04412]).value, dtype=float)
        self.T_livox_body = make_transform(np.eye(3), -lidar_in_imu)

        # Correction to the URDF LiDAR mount, about livox_frame's own axes.
        mount_rpy = self.declare_parameter(
            'lidar_mount_rpy_offset', [0.0, 0.0, 0.0]).value
        self.T_mount_offset = make_transform(
            Rotation.from_euler('xyz', mount_rpy).as_matrix(), np.zeros(3))

        self.sole_offset = self.declare_parameter('sole_offset', 0.0).value
        self.init_duration = self.declare_parameter('init_duration', 1.0).value
        self.gyro_still_threshold = self.declare_parameter(
            'gyro_still_threshold', 0.05).value
        self.velocity_alpha = self.declare_parameter('velocity_alpha', 0.5).value
        self.tf_timeout = Duration(
            seconds=self.declare_parameter('tf_timeout', 0.05).value)

        # The listener spins its own thread, so the blocking lookups in
        # the odometry callback do not stop TF from being received.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.fastlio_sub = self.create_subscription(
            Odometry, fastlio_odom_topic, self.fastlio_callback, 20)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)

        # Stage 1 state.
        self.T_odom_cam = None
        self.accel_samples = []
        self.accel_window_start = None
        self.still_long_enough = False

        # Stage 2 velocity state: (stamp, x, y, yaw) and smoothed twist.
        self.previous = None
        self.velocity = np.zeros(3)

        self.get_logger().info(
            'PM01 odom publisher started. Keep the robot still with both '
            'feet on the floor until odom is initialised.')

    # ------------------------------------------------------------------
    # Stage 1 input: gravity direction while standing still
    # ------------------------------------------------------------------

    def imu_callback(self, msg):
        stamp = stamp_to_sec(msg.header.stamp)
        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])

        if np.linalg.norm(gyro) > self.gyro_still_threshold:
            # Robot is moving: start the stillness window again.
            self.accel_samples.clear()
            self.accel_window_start = None
            self.still_long_enough = False
            return

        if self.accel_window_start is None:
            self.accel_window_start = stamp

        self.accel_samples.append([msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z])

        if stamp - self.accel_window_start >= self.init_duration:
            self.still_long_enough = True

    # ------------------------------------------------------------------
    # Every FAST-LIO update
    # ------------------------------------------------------------------

    def fastlio_callback(self, msg):
        # FAST-LIO pose: camera_init -> body.
        T_cam_body = transform_from_msg(msg.pose.pose.position,
                                        msg.pose.pose.orientation)

        # Robot kinematics at the same instant: LINK_BASE -> livox_frame.
        # Includes the waist yaw joint and the URDF LiDAR mount.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.lidar_frame,
                Time.from_msg(msg.header.stamp), self.tf_timeout)
        except TransformException as ex:
            self.get_logger().warn(
                f'Waiting for {self.base_frame} -> {self.lidar_frame} at the '
                f'FAST-LIO timestamp: {ex}', throttle_duration_sec=2.0)
            return

        T_base_livox = transform_from_msg(
            tf.transform.translation, tf.transform.rotation) @ self.T_mount_offset

        # Pelvis -> IMU, then the pelvis pose in FAST-LIO's world.
        T_base_body = T_base_livox @ self.T_livox_body
        T_cam_base = T_cam_body @ invert(T_base_body)

        if self.T_odom_cam is None:
            self.initialise_odom(msg, T_cam_body, T_cam_base)
            return

        self.publish_odometry(msg.header.stamp, self.T_odom_cam @ T_cam_base)

    # ------------------------------------------------------------------
    # Stage 1: define odom once, with the robot standing still
    # ------------------------------------------------------------------

    def initialise_odom(self, msg, T_cam_body, T_cam_base):
        if not self.still_long_enough:
            self.get_logger().info(
                'Waiting for the robot to stand still for '
                f'{self.init_duration:.1f} s ({self.imu_sub.topic_name})',
                throttle_duration_sec=1.0)
            return

        # Foot positions relative to the pelvis, for the floor height.
        try:
            feet = [
                transform_from_msg(
                    *self.lookup(self.base_frame, foot, msg.header.stamp))[:3, 3]
                for foot in self.foot_frames
            ]
        except TransformException as ex:
            self.get_logger().warn(
                f'Waiting for foot TF: {ex}', throttle_duration_sec=2.0)
            return

        # 1. "Up" in camera_init. At rest the accelerometer measures the
        #    reaction to gravity, which points up.
        mean_accel = np.mean(self.accel_samples, axis=0)
        up_cam = T_cam_body[:3, :3] @ mean_accel
        up_cam /= np.linalg.norm(up_cam)

        # 2. Level: turn "up" into +z.
        R_level = rotation_aligning(up_cam, np.array([0.0, 0.0, 1.0]))

        # 3. Heading: odom x points the way the pelvis faces.
        yaw0 = heading(R_level @ T_cam_base[:3, :3])
        R_odom_cam = rot_z(-yaw0) @ R_level

        # 4. Floor: the lower foot, in odom orientation, relative to camera_init.
        foot_heights = [
            (R_odom_cam @ (T_cam_base @ np.append(foot, 1.0))[:3])[2]
            for foot in feet
        ]
        z_floor = min(foot_heights) - self.sole_offset

        # 5. Origin on the floor directly under the pelvis.
        pelvis = R_odom_cam @ T_cam_base[:3, 3]
        t_odom_cam = -np.array([pelvis[0], pelvis[1], z_floor])

        self.T_odom_cam = make_transform(R_odom_cam, t_odom_cam)
        self.static_broadcaster.sendTransform(to_transform_stamped(
            self.T_odom_cam, msg.header.stamp, self.odom_frame,
            msg.header.frame_id))

        # Gravity is no longer needed.
        self.destroy_subscription(self.imu_sub)
        self.accel_samples.clear()

        T_odom_base = self.T_odom_cam @ T_cam_base
        roll, pitch, _ = Rotation.from_matrix(
            T_odom_base[:3, :3]).as_euler('xyz', degrees=True)
        x, y, z, w = Rotation.from_matrix(R_odom_cam).as_quat()
        tilt = math.degrees(math.acos(np.clip(
            np.dot(mean_accel / np.linalg.norm(mean_accel), [1.0, 0.0, 0.0]),
            -1.0, 1.0)))
        self.get_logger().info(
            'odom initialised.\n'
            f'  {self.odom_frame} -> {msg.header.frame_id}: '
            f't = ({t_odom_cam[0]:.3f}, {t_odom_cam[1]:.3f}, {t_odom_cam[2]:.3f}) m, '
            f'q xyzw = ({x:.3f}, {y:.3f}, {z:.3f}, {w:.3f})\n'
            f'  pelvis height {T_odom_base[2, 3]:.3f} m, '
            f'roll {roll:.2f} deg, pitch {pitch:.2f} deg\n'
            f'  LiDAR +X tilt from vertical {tilt:.2f} deg '
            f'({len(feet)} feet, floor {-z_floor:.3f} m below {msg.header.frame_id})')

        self.publish_odometry(msg.header.stamp, T_odom_base)

    def lookup(self, target, source, stamp):
        tf = self.tf_buffer.lookup_transform(
            target, source, Time.from_msg(stamp), self.tf_timeout)
        return tf.transform.translation, tf.transform.rotation

    # ------------------------------------------------------------------
    # Stage 2: split the pelvis pose and publish
    # ------------------------------------------------------------------

    def publish_odometry(self, stamp, T_odom_base):
        x, y = T_odom_base[0, 3], T_odom_base[1, 3]
        yaw = heading(T_odom_base[:3, :3])

        # odom -> base_footprint: on the floor, heading only.
        T_odom_fp = make_transform(rot_z(yaw), [x, y, 0.0])
        # base_footprint -> LINK_BASE: height, roll and pitch.
        T_fp_base = invert(T_odom_fp) @ T_odom_base

        self.update_velocity(stamp_to_sec(stamp), x, y, yaw)

        self.tf_broadcaster.sendTransform([
            to_transform_stamped(T_odom_fp, stamp,
                                 self.odom_frame, self.footprint_frame),
            to_transform_stamped(T_fp_base, stamp,
                                 self.footprint_frame, self.base_frame),
        ])

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.footprint_frame

        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)

        odom.twist.twist.linear.x = float(self.velocity[0])
        odom.twist.twist.linear.y = float(self.velocity[1])
        odom.twist.twist.angular.z = float(self.velocity[2])

        # Placeholder covariances, NOT calibrated. z, roll and pitch are
        # not estimated in this planar odometry.
        planar = [0.01, 0.01, 1e6, 1e6, 1e6, 0.01]
        twist = [0.05, 0.05, 1e6, 1e6, 1e6, 0.05]
        for i in range(6):
            odom.pose.covariance[i * 7] = planar[i]
            odom.twist.covariance[i * 7] = twist[i]

        self.odom_pub.publish(odom)

    def update_velocity(self, stamp, x, y, yaw):
        previous = self.previous
        self.previous = (stamp, x, y, yaw)

        if previous is None:
            return

        dt = stamp - previous[0]
        if dt <= 0.0 or dt > 0.5:
            self.velocity[:] = 0.0
            return

        # World-frame velocity, rotated into base_footprint.
        vx_odom = (x - previous[1]) / dt
        vy_odom = (y - previous[2]) / dt
        wz = wrap_angle(yaw - previous[3]) / dt

        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        measured = np.array([
            cos_yaw * vx_odom + sin_yaw * vy_odom,
            -sin_yaw * vx_odom + cos_yaw * vy_odom,
            wz,
        ])

        self.velocity = (self.velocity_alpha * measured
                         + (1.0 - self.velocity_alpha) * self.velocity)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
