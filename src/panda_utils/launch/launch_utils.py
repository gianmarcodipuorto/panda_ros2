from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    controller_kp = DeclareLaunchArgument(
        'controller_kp',
        default_value='3750.0',
        description='Controller Kp'
    )

    controller_kd = DeclareLaunchArgument(
        'controller_kd',
        default_value='750.0',
        description='Controller Kd'
    )

    control_rate = DeclareLaunchArgument(
        'controller_rate',
        default_value='1000.0',
        description='Controller PD + gravity rate'
    )

    clik_ts = DeclareLaunchArgument(
        'clik_ts',
        default_value='0.01',
        description='CLIK related sample time'
    )

    clik_gamma = DeclareLaunchArgument(
        'clik_gamma',
        default_value='0.5',
        description='CLIK related gamma gain (not divided by ts)'
    )

    clamp_effort_control = DeclareLaunchArgument(
        'clamp_effort_control',
        default_value='True',
        description='Clamp effort control from controller'
    )

    # jacobian_calculator = Node(
    #     package='panda_utils',
    #     executable='calc_jacobian_server',
    #     name='jacobian_calculator',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #     }],
    # )
    # forward_kine = Node(
    #     package='panda_utils',
    #     executable='calc_forward_kine_server',
    #     name='forward_kine',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #     }],
    # )
    pos_cmds_joints = Node(
        package='panda_utils',
        executable='send_joints_cmd_server',
        name='pos_cmds_joints',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    effort_cmd_server = Node(
        package='panda_utils',
        executable='send_joints_effort_server',
        name='effort_cmd_server',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    joint_traj_server = Node(
        package='panda_utils',
        executable='joint_traj',
        name='joint_traj_server',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    cart_traj_server = Node(
        package='panda_utils',
        executable='cart_traj',
        name='cart_traj_server',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    clik_cmd_pub = Node(
        package='panda_utils',
        executable='clik_cmd_pub',
        name='clik_cmd_pub',
        parameters=[{
            'use_sim_time': use_sim_time,
            'ts': LaunchConfiguration('clik_ts'),
            'gamma': LaunchConfiguration('clik_gamma'),
        }],
    )
    pd_grav_controller = Node(
        package='panda_utils',
        executable='pd_grav_controller',
        name='pd_grav_controller',
        parameters=[{
            'use_sim_time': use_sim_time,
            'Kp': LaunchConfiguration('controller_kp'),
            'Kd': LaunchConfiguration('controller_kd'),
            'control_freq': LaunchConfiguration('controller_rate'),
            'clamp': LaunchConfiguration('clamp_effort_control')
        }],
    )

    return LaunchDescription([
        controller_kp,
        controller_kd,
        control_rate,
        clamp_effort_control,
        clik_ts,
        clik_gamma,
        pos_cmds_joints,
        effort_cmd_server,
        joint_traj_server,
        cart_traj_server,
        clik_cmd_pub,
        pd_grav_controller
    ])
