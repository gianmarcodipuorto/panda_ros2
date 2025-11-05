from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world_to_kinect_depth',
                default_value='[0.0, 0.0, 2.1, -1.57, 0.0, -1.57, world, kinect_depth]',
                description='World to kinect transform'
            ),
            # DeclareLaunchArgument(
            #     'depth_to_rgb',
            #     default_value='[-0.0254, -0.00013, -0.00218,-1.57, 0.0, -1.57, world, kinect_depth]',
            #     description='Base to world transform'
            # ),
            Node(
                package="kinect_ros2",
                executable="kinect_ros2_node",
                name="kinect_ros2",
            ),
            ComposableNodeContainer(
                name='depth_image_proc_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='depth_image_proc',
                        plugin='depth_image_proc::RegisterNode',
                        name='register_depth_node',
                        remappings=[
                            ('depth/image_rect', '/depth/image_raw'),
                            ('rgb/camera_info', '/camera_info'),
                        ],
                    ),
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_world_rgb',
                arguments=[
                    # '0.0', '0.0', '2.1',        # x y z
                    '-0.725', '-0.265', '2.13',        # x y z
                    # '-1.57', '0.0', '-1.57',        # roll pitch yaw (in radians)
                    # '0.857', '0.3613', '-0.3875',        # roll pitch yaw (in radians)
                    # '-0.7838', '0.0003', '-2.093', working one
                    '-0.784650', '0.000272', '-1.9190',
                    'world', 'kinect_depth'  # parent_frame child_frame
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_depth_to_rgb',
                arguments=[
                    '-0.025518', '0.0005438', '0.0018004',
                    '0.0025', '-0.0102', '-0.0018',
                    'kinect_depth', 'kinect_rgb'
                ]
            ),
        ]
    )
