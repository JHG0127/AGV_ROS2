import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # autostart = LaunchConfiguration('autostart', default='true')
    # lifecycle_nodes = ['controller_server',
    #                    'planner_server',
    #                    'recoveries_server',
    #                    'bt_navigator',
    #                    'waypoint_follower',
    #                    'amcl']
    
    map_dir = LaunchConfiguration(
        'map_dir', 
        default=os.path.join(
            get_package_share_directory('myagv_navigation2'),
            'map',
            'my_map3.yaml'
        ))
    param_fime_name = "myagv.yaml"
    param_dir = LaunchConfiguration(
        'param_file',
        default=os.path.join(
            get_package_share_directory('myagv_navigation2'),
            'param',
            param_fime_name))
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    
    rviz_config_dir = os.path.join(
     	get_package_share_directory('myagv_navigation2'),
     	'rviz',
     	'nav2_config_object_following.rviz')
    # rviz_config_dir = os.path.join(
    #  	get_package_share_directory('nav2_bringup'),
    #  	'rviz',
    #  	'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_navigation',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time},
        #                 {'autostart': autostart},
        #                 {'node_names': lifecycle_nodes}]),
    ])
