from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(

        ),

        
    ])

def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('custom_dwa_planner'),
        'config',
        'params.yaml'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('custom_dwa_planner'),
        'rviz',
        'default_view.rviz'
    )
    
    # Include Gazebo + TurtleBot3 world
    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            )
        )
    )

    planner_node = Node(
        package='custom_dwa_planner',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        parameters=[params_file],
        # remappings=[
        #     ('/cmd_vel', '/cmd_vel'),
        #     ('/odom', '/odom'),
        #     ('/scan', '/scan'),
        #     ('/move_base_simple/goal', '/move_base_simple/goal')
        # ]
    )

    # RViz2 default
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        turtlebot_launch,
        rviz_node,
        planner_node
    ])
