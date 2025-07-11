from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    no_depth = DeclareLaunchArgument(
        'no_depth',
        default_value='false',
        description="Don't estimate depth from depth image")

    process_noise = DeclareLaunchArgument(
        'process_noise',
        default_value='20.0',
        description="Process scalar noise value")

    measurement_noise = DeclareLaunchArgument(
        'measurement_noise',
        default_value='50.0',
        description="Measurement scalar noise value")

    skeleton_tracking = Node(
        package='image_processing',
        executable='skeleton_track',
        name='skeleton_track',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'no_depth': LaunchConfiguration('no_depth'),
            'process_noise': LaunchConfiguration('process_noise'),
            'measurement_noise': LaunchConfiguration('measurement_noise'),
        }],
    )

    return LaunchDescription([
        measurement_noise,
        process_noise,
        use_sim_time,
        no_depth,
        skeleton_tracking,
    ])
