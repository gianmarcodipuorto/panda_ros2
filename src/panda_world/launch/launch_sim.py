# Copyright 2024 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FileContent, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():

    # ''use_sim_time'' is used to have ros2 use /clock topic for the time source
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_panda_world = get_package_share_directory('panda_world')

    urdf = os.path.join(get_package_share_directory('panda_world'),
                        'models', 'panda', 'panda_fr3_rviz.urdf')
    default_rviz_config = os.path.join(
        get_package_share_directory('panda_world'), 'config', 'config.rviz')

    with open(urdf, 'r') as file:
        robot_description = file.read()

    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    rviz_launch_arg = DeclareLaunchArgument('rviz', default_value='false',
                                            description='Open RViz.')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        # FIXME: Generate new RViz config once this demo is usable again
        # arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'depth_camera.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', default_rviz_config]
    )

    robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        arguments=[urdf])

    panda_fr3_model = PathJoinSubstitution([
        pkg_panda_world,
        'models',
        'panda',
        'panda_fr3.urdf'
    ])

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_panda_world,
            'worlds',
            'empty_world.sdf'
        ])}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
                'config_file': os.path.join(pkg_panda_world, 'config', 'bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    joint_state_publisher_patched = Node(
        package='panda_world',
        executable='joint_state_publisher_effort_patcher',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    torque_publisher = Node(
        package='panda_world',
        executable='torque_sensor_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    world = LaunchConfiguration('world')
    file = LaunchConfiguration('file')
    model_string = LaunchConfiguration('model_string')
    topic = LaunchConfiguration('topic')
    entity_name = LaunchConfiguration('entity_name')
    allow_renaming = LaunchConfiguration('allow_renaming')
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.0')
    roll = LaunchConfiguration('R', default='0.0')
    pitch = LaunchConfiguration('P', default='0.0')
    yaw = LaunchConfiguration('Y', default='0.0')

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text='panda_world'),
        description='World name')
    declare_file_cmd = DeclareLaunchArgument(
        'file', default_value=panda_fr3_model,
        description='SDF/URDF filename of model')
    declare_model_string_cmd = DeclareLaunchArgument(
        'model_string',
        default_value='',
        description='XML(SDF) string',
    )
    declare_topic_cmd = DeclareLaunchArgument(
        'topic', default_value=TextSubstitution(text=''),
        description='Get XML from this topic'
    )
    declare_entity_name_cmd = DeclareLaunchArgument(
        'entity_name', default_value=TextSubstitution(text='panda'),
        description='Name of the entity'
    )
    declare_allow_renaming_cmd = DeclareLaunchArgument(
        'allow_renaming', default_value='False',
        description='Whether the entity allows renaming or not'
    )

    load_nodes = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'world': world,
                     'file': file,
                     'string': model_string,
                     'topic': topic,
                     'name': entity_name,
                     'allow_renaming': allow_renaming,
                     'x': x,
                     'y': y,
                     'z': z,
                     'R': roll,
                     'P': pitch,
                     'Y': yaw,
                     }],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_file_cmd)
    ld.add_action(declare_model_string_cmd)
    ld.add_action(declare_topic_cmd)
    ld.add_action(declare_entity_name_cmd)
    ld.add_action(declare_allow_renaming_cmd)
    # Add the actions to launch all of the create nodes
    ld.add_action(gz_sim)
    ld.add_action(load_nodes)
    ld.add_action(robot_state)
    ld.add_action(bridge)
    ld.add_action(joint_state_publisher_patched)
    ld.add_action(torque_publisher)
    ld.add_action(rviz_launch_arg)
    ld.add_action(rviz)

    return ld
