
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Publishes odom -> camera_init (static, computed at startup),
    # odom -> base_footprint, base_footprint -> LINK_BASE and /odom.
    #
    # Start FAST-LIO first, keep the robot still with both feet on the
    # floor until "odom initialised" is logged, and restart this node
    # whenever FAST-LIO restarts. Do not run nav_bridge at the same time.
    #
    # See README.md in this package for the frames and the math.
    odom_publisher = Node(
        package='pm01_nav_bridge',
        executable='odom_publisher_node.py',
        name='pm01_odom_publisher',
        parameters=[{
            # Keep equal to extrinsic_T in pm01_mapping/FAST_LIO/config/pm01.yaml.
            'lidar_in_imu': [-0.011, -0.02329, 0.04412],

            # Correction to the URDF LiDAR mount (rad, about livox_frame axes).
            'lidar_mount_rpy_offset': [0.0, 0.0, 0.0],

            # Distance from LINK_FOOT_L/R down to the floor.
            'sole_offset': 0.0,

            'init_duration': 1.0,
            'gyro_still_threshold': 0.05,
            'velocity_alpha': 0.5,
        }],
        output='screen',
    )

    return LaunchDescription([
        odom_publisher,
    ])
