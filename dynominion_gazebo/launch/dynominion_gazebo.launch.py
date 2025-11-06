# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world = LaunchConfiguration('world',default="cafe.world")
    config_file = LaunchConfiguration('config',default="gz_bridge.yaml")
    ekf_file = LaunchConfiguration('ekf_file',default="ekf.yaml")

    world_path = PathJoinSubstitution([
        FindPackageShare("dynominion_gazebo"),
        "worlds",
        world
    ])
    
    config_path = PathJoinSubstitution([FindPackageShare("dynominion_gazebo"),
        "config",
        config_file
    ])

    def robot_state_publisher(context):
        # Get URDF or SDF via xacro
        xacro_file_path = PathJoinSubstitution([
        FindPackageShare('dynominion_gazebo'),
            'urdf',
            'dynominion.urdf.xacro'
        ])

        robot_description = {"robot_description": Command(['xacro ', xacro_file_path]), "use_sim_time": use_sim_time}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        )
        return [node_robot_state_publisher]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('dynominion_gazebo'),
            'config',
            'diff_drive_controller.yaml',
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'diff_drive', '-allow_renaming', 'true',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '2.5'
                   ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            robot_controllers,
            ],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file':config_path,  "use_sim_time": use_sim_time}],
        output='screen'
    )

    odom_modifier = Node(
        package='dynominion_gazebo',
        executable='odom_modifier.py',
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    ld = LaunchDescription([
        # Launch gazebo environment
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([FindPackageShare('dynominion_gazebo'), 'models'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 ', world_path])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),
        bridge,
        odom_modifier,
        # joint_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        
        DeclareLaunchArgument(
            'world',
            default_value='cafe.world',
            description='World file name (must be in dynominion_gazebo/worlds)'
        ),

        DeclareLaunchArgument(
            'config',
            default_value='gz_bridge.yaml',
            description='gz_bridge file name (must be in dynominion_gazebo/config)'
        ),
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
