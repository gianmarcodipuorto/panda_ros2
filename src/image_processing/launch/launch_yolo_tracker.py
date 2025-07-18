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

    ma_confidence_threshold = DeclareLaunchArgument(
        'ma_confidence_threshold',
        default_value='0.40',
        description="Moving average confidence threshold to consider the valid keypoints")

    hallucination_threshold = DeclareLaunchArgument(
        'hallucination_threshold',
        default_value='0.25',
        description="Confidence under which the keypoint is not considered")

    single_keypoint_ma_confidence_threshold = DeclareLaunchArgument(
        'single_keypoint_ma_confidence_threshold',
        default_value='0.35',
        description="Single keypoint moving average confidence threshold to consider the keypoint valid")

    ma_window_size = DeclareLaunchArgument(
        'MA_window_size',
        default_value='40',
        description="Windows size for moving average")

    skeleton_tracking = Node(
        package='image_processing',
        executable='skeleton_track_yolo',
        name='skeleton_track_yolo',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'no_depth': LaunchConfiguration('no_depth'),
            'process_noise': LaunchConfiguration('process_noise'),
            'measurement_noise': LaunchConfiguration('measurement_noise'),
            'ma_confidence_threshold': LaunchConfiguration('ma_confidence_threshold'),
            'hallucination_threshold': LaunchConfiguration('hallucination_threshold'),
            'single_keypoint_ma_confidence_threshold': LaunchConfiguration('single_keypoint_ma_confidence_threshold'),
            'MA_window_size': LaunchConfiguration('MA_window_size'),
        }],
    )

    return LaunchDescription([
        measurement_noise,
        process_noise,
        use_sim_time,
        no_depth,
        ma_window_size,
        ma_confidence_threshold,
        hallucination_threshold,
        single_keypoint_ma_confidence_threshold,
        skeleton_tracking,
    ])
