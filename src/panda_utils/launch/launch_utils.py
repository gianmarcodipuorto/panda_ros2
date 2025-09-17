from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    use_sim_time = DeclareLaunchArgument(
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

    controller_md = DeclareLaunchArgument(
        'controller_md',
        default_value='100.0',
        description='Controller Md'
    )

    lambda_pinv = DeclareLaunchArgument(
        'lambda',
        default_value='0.01',
        description='Lambda value for damped least square jacobian pseudo-inverse'
    )
    k_max = DeclareLaunchArgument(
        'k_max',
        default_value='2.0',
        description='Max value for quadratic based lambda handling'
    )
    eps = DeclareLaunchArgument(
        'eps',
        default_value='0.1',
        description='Minimum value for quadratic based lambda handling'
    )

    control_rate = DeclareLaunchArgument(
        'controller_rate',
        default_value='1000.0',
        description='Controller PD + gravity rate'
    )

    loop_rate_freq = DeclareLaunchArgument(
        'loop_rate_freq',
        default_value='1000.0',
        description='Trajectory server rate frequency'
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

    use_robot = DeclareLaunchArgument(
        'use_robot',
        default_value='False',
        description='Use fr3 real robot (set also robot ip)'
    )

    use_franka_sim = DeclareLaunchArgument(
        'use_franka_sim',
        default_value='False',
        description='Use panda real robot library (set also robot ip)'
    )

    robot_ip = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.0',
        description='fr3 robot ip'
    )

    world_base_link = DeclareLaunchArgument(
        'world_base_link',
        default_value='[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]',
        description='Base to world transform'
    )

    alpha = DeclareLaunchArgument(
        'alpha',
        default_value='30.0',
        description='Paramater used for computations of DLS jacobian based on sigma min'
    )

    task_gain = DeclareLaunchArgument(
        'task_gain',
        default_value='1.0',
        description='Secondary task gain'
    )
    effort_cmd_server = Node(
        package='panda_utils',
        executable='send_joints_effort_server',
        name='effort_cmd_server',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    joint_traj_server = Node(
        package='panda_utils',
        executable='joint_traj',
        name='joint_traj_server',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    cart_traj_server = Node(
        package='panda_utils',
        executable='cart_traj',
        name='cart_traj_server',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 'loop_rate_freq': loop_rate_freq,
        }],
    )
    stop_traj_server = Node(
        package='panda_utils',
        executable='stop_traj',
        name='stop_traj_server',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 'loop_rate_freq': loop_rate_freq,
        }],
    )
    loop_cart_traj_server = Node(
        package='panda_utils',
        executable='loop_cart_traj',
        name='loop_cart_traj_server',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 'loop_rate_freq': loop_rate_freq,
        }],
    )
    clik_cmd_pub = Node(
        package='panda_utils',
        executable='clik_cmd_pub',
        name='clik_cmd_pub',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'ts': LaunchConfiguration('clik_ts'),
            'gamma': LaunchConfiguration('clik_gamma'),
        }],
    )
    inverse_dynamics_controller = Node(
        package='panda_utils',
        executable='inverse_dynamics_controller',
        name='inverse_dynamics_controller',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'Kp': LaunchConfiguration('controller_kp'),
            'Kd': LaunchConfiguration('controller_kd'),
            'Md': LaunchConfiguration('controller_md'),
            'alpha': LaunchConfiguration('alpha'),
            'control_freq': LaunchConfiguration('controller_rate'),
            'clamp': LaunchConfiguration('clamp_effort_control'),
            'use_robot': LaunchConfiguration('use_robot'),
            'use_franka_sim': LaunchConfiguration('use_franka_sim'),
            'robot_ip': LaunchConfiguration('robot_ip')

        }],
    )
    impedance_controller = Node(
        package='panda_utils',
        executable='impedance_controller',
        name='impedance_controller',
        # prefix=["gdbserver localhost:3000"],
        prefix=['kitty -e gdb -ex run --args'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'Kp': LaunchConfiguration('controller_kp'),
            'Kd': LaunchConfiguration('controller_kd'),
            'Md': LaunchConfiguration('controller_md'),
            'lambda': LaunchConfiguration('lambda'),
            'k_max': LaunchConfiguration('k_max'),
            'eps': LaunchConfiguration('eps'),
            'task_gain': LaunchConfiguration('task_gain'),
            'alpha': LaunchConfiguration('alpha'),
            'control_freq': LaunchConfiguration('controller_rate'),
            'clamp': LaunchConfiguration('clamp_effort_control'),
            'use_robot': LaunchConfiguration('use_robot'),
            'use_franka_sim': LaunchConfiguration('use_franka_sim'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'world_base_link': LaunchConfiguration('world_base_link')
        }],
    )
    # publish_franks_tfs = Node(
    #     package='panda_utils',
    #     executable='publish_franks_tfs',
    #     name='publish_franks_tfs',
    #     parameters=[{
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'robot_ip': LaunchConfiguration('robot_ip'),
    #         'world_base_link': LaunchConfiguration('world_base_link')
    #     }],
    # )
    # external_force_estimator = Node(
    #     package='panda_utils',
    #     executable='estimate_external_forces',
    #     name='estimate_external_forces',
    #     parameters=[{
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'robot_ip': LaunchConfiguration('robot_ip'),
    #         'world_base_link': LaunchConfiguration('world_base_link')
    #     }],
    # )
    publish_tfs_and_estimate_forces = Node(
        package='panda_utils',
        executable='estimate_hand_external_force',
        name='estimate_hand_external_force',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'world_base_link': LaunchConfiguration('world_base_link')
        }],
    )

    return LaunchDescription([
        use_sim_time,
        controller_kp,
        controller_kd,
        controller_md,
        lambda_pinv,
        eps,
        k_max,
        control_rate,
        loop_rate_freq,
        clamp_effort_control,
        use_robot,
        use_franka_sim,
        robot_ip,
        clik_ts,
        clik_gamma,
        alpha,
        task_gain,
        world_base_link,
        effort_cmd_server,
        joint_traj_server,
        cart_traj_server,
        stop_traj_server,
        loop_cart_traj_server,
        clik_cmd_pub,
        inverse_dynamics_controller,
        impedance_controller,
        # publish_franks_tfs,
        # external_force_estimator,
        publish_tfs_and_estimate_forces
    ])
