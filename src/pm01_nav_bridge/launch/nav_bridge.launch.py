
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Gravity-aligned navigation reference.
    #
    # The earlier diagnostic used:
    # camera_init -> nav_test with +90 degrees pitch.
    #
    # Here we instead use its inverse:
    # odom -> camera_init with -90 degrees pitch.
    #
    # Translation is zero for initial validation.
    odom_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_camera_init',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '-1.57079632679',
            '--yaw', '0.0',
            '--frame-id', 'odom',
            '--child-frame-id', 'camera_init',
        ],
        output='screen',
    )

    nav_bridge = Node(
        package='pm01_nav_bridge',
        executable='nav_bridge',
        name='pm01_nav_bridge',
        parameters=[{
            'mount_x': 0.0,
            'mount_y': 0.0,
            'mount_z': 0.2191,
        }],
        output='screen',
    )

    return LaunchDescription([
        odom_to_camera_init,
        nav_bridge,
    ])