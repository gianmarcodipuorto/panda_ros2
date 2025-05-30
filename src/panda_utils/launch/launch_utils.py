from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    jacobian_calculator = Node(
        package='panda_utils',
        executable='calc_jacobian_server',
        parameters=[{
            'use_sim_time': True,
        }],
    )
    forward_kine = Node(
        package='panda_utils',
        executable='calc_forward_kine_server',
        parameters=[{
            'use_sim_time': True,
        }],
    )
    pos_cmds_joints = Node(
        package='panda_utils',
        executable='send_joints_cmd_server',
        parameters=[{
            'use_sim_time': True,
        }],
    )
    effort_cmd_server = Node(
        package='panda_utils',
        executable='send_joints_effort_server',
        parameters=[{
            'use_sim_time': True,
        }],
    )
    joint_traj_server = Node(
        package='panda_utils',
        executable='joint_traj',
        parameters=[{
            'use_sim_time': False,
        }],
    )
    cart_traj_server = Node(
        package='panda_utils',
        executable='cart_traj',
        parameters=[{
            'use_sim_time': False,
        }],
    )
    clik_cmd_pub = Node(
        package='panda_utils',
        executable='clik_cmd_pub',
        parameters=[{
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        forward_kine,
        jacobian_calculator,
        pos_cmds_joints,
        effort_cmd_server,
        joint_traj_server,
        cart_traj_server,
        clik_cmd_pub
    ])
