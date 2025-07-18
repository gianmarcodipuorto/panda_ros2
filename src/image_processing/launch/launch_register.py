from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    container = ComposableNodeContainer(
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
    )

    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_depth_to_rgb',
        arguments=['-0.0254', '-0.00013', '-0.00218', '0', '0', '0', 'kinect_depth', 'kinect_rgb']
    )

    return LaunchDescription([container, tf_node])

