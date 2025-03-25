import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node

def generate_launch_description():

    # Command line arguments
    background_r_launch_arg = DeclareLaunchArgument("background_r", default_value=TextSubstitution(text="0"))
    background_g_launch_arg = DeclareLaunchArgument("background_g", default_value=TextSubstitution(text="0"))
    background_b_launch_arg = DeclareLaunchArgument("background_b", default_value=TextSubstitution(text="0"))
    
    chatter_ns_launch_arg = DeclareLaunchArgument("chatter_ns", default_value=TextSubstitution(text="my/chatter/ns"))
    
    # Include another launch file
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('demo_nodes_cpp'),
                'launch/topics/talker_listener.launch.py'))
        )
    # Include another launch file in the chatter namespace
    launch_include_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('chatter_ns'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('demo_nodes_cpp'),
                        'launch/topics/talker_listerner.launch.py'))
            ),
        ]
    )
    
    # Start turtlesim node in turtlesim1 namespace
    turtlesim_node = Node(
        package='turtlesim',
        namespace='turtlesim1',
        executable='turtlesim_node',
        name='sim'
    )
    
    # Start another turtlesim node in the turtlesim2 namespace and use args to set parameters
    turtlesim_node_with_parameters = Node(
        package='turtlesim',
        namespace='turtlesim2',
        executable='turtlesim_node',
        name='sim',
        parameters=[{
            "background_r":LaunchConfiguration('background_r'),
            "background_g":LaunchConfiguration('background_g'),
            "background_b":LaunchConfiguration('background_b')
        }]
    )
    
    # Perform remap so both turtles listen to the same command topic
    forward_turtlesim_commands_to_second_turtlesim_node = Node(
        package='turtlesim',
        executable='mimic',
        name='mimic',
        remappings=[
            ('/input/pose', '/turtlesim1/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        background_r_launch_arg,
        background_g_launch_arg,
        background_b_launch_arg,
        chatter_ns_launch_arg,
        turtlesim_node,
        turtlesim_node_with_parameters,
        forward_turtlesim_commands_to_second_turtlesim_node,
    ])

