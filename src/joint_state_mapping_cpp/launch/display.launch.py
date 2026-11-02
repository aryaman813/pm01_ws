import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description_share = get_package_share_directory('mujoco_simulator')

    urdf_path = os.path.join(
        description_share,
        'assets',
        'resource',
        'robot',
        'pm_v2',
        'urdf',
        'serial_pm_v2.urdf',
    )
    
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # The simulation URDF uses a resource directory as a package name.
    # Resolve meshes through the ROS package that actually installs them.
    robot_desc = robot_desc.replace(
        'package://resource/',
        'package://mujoco_simulator/assets/resource/',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_robot_state_publisher',
            default_value='false',
            description='Publish the local URDF only when the robot does not provide it',
        ),
        DeclareLaunchArgument(
            'use_joint_state_mapper',
            default_value='false',
            description='Convert /hardware/joint_state locally when the robot does not publish /joint_states',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            condition=IfCondition(LaunchConfiguration('use_robot_state_publisher')),
            output='screen',
            parameters=[
                {
                    'robot_description': robot_desc, 
                    # 'use_sim_time': True
                }]
        ),

        Node(
            package='joint_state_mapping',
            executable='joint_state_mapper',
            name='joint_state_mapper',
            condition=IfCondition(LaunchConfiguration('use_joint_state_mapper')),
            output='screen',
            parameters=[{
                
                # 'use_sim_time': True
                         
                         }],
        ),
    ])


    
