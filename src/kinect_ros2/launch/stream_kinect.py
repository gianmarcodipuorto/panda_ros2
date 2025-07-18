from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kinect_ros2",
                executable="kinect_ros2_node",
                name="kinect_ros2",
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_world_rgb',
                arguments=[
                    '0.0', '0.0', '2.1',        # x y z
                    '0.0', '0.0', '-1.5',        # roll pitch yaw (in radians)
                    'world', 'kinect_depth'  # parent_frame child_frame
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_depth_to_rgb',
                arguments=[
                    '-0.0254', '-0.00013', '-0.00218',
                    '0', '0', '0',
                    'kinect_depth', 'kinect_rgb'
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_world_rgb_optical',
                arguments=[
                    '0.0', '0.0', '2.1',        # x y z
                    '0.0', '0.0', '-1.5',        # roll pitch yaw (in radians)
                    'world', 'depth_optical_frame'  # parent_frame child_frame
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_world_depth_optical',
                arguments=[
                    '-0.0254', '-0.00013', '-0.00218',
                    '0', '0', '0',
                    'depth_optical_frame', 'rgb_optical_frame'
                ]
            )
        ]
    )
