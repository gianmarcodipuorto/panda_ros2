from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panda_world'),
                'launch',
                'launch_sim.py'
            ])
        ]),
        launch_arguments={
        }.items()
    )

    utils_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panda_utils'),
                'launch',
                'launch_utils.py'
            ])
        ]),
        launch_arguments={
        }.items()
    )

    yolo_recognition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'),
                'launch',
                'yolov8.launch.py'
            ])
        ]),
        launch_arguments={
        }.items()
    )

    image_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('image_processing'),
                'launch',
                'launch_yolo_tracker.py'
            ])
        ]),
        launch_arguments={
        }.items()
    )

    kinect_ros2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kinect_ros2'),
                'launch',
                'stream_kinect.py'
            ])
        ]),
        launch_arguments={
        }.items()
    )

    return LaunchDescription([
        # Launch arguments

        # Launch files
        world_launch,
        utils_launch,
        yolo_recognition,
        image_processing,
        kinect_ros2,
    ])
