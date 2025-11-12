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

    controller_kp_rot = DeclareLaunchArgument(
        'controller_kp_rot',
        default_value='200.0',
        description='Controller Kp rotational part'
    )

    controller_kd_rot = DeclareLaunchArgument(
        'controller_kd_rot',
        default_value='10.0',
        description='Controller Kd rotational part'
    )

    controller_md_rot = DeclareLaunchArgument(
        'controller_md_rot',
        default_value='1.0',
        description='Controller Md rotational part'
    )

    control_rate = DeclareLaunchArgument(
        'controller_rate',
        default_value='1000.0',
        description='Controller rate'
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

    robot_ip = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.0',
        description='fr3 robot ip'
    )

    world_base_link = DeclareLaunchArgument(
        'world_base_link',
        default_value='[0.25, 0.17, 0.845, 1.0, 0.0, 0.0, 0.0]',
        description='Base to world transform'
    )

    safe_joint_speed = DeclareLaunchArgument(
        'safe_joint_speed',
        default_value='0.7',
        description='Maximum joint speed allowed'
    )

    safe_effort_perc = DeclareLaunchArgument(
        'safe_effort_perc',
        default_value='0.5',
        description='Maximum percentage of maximum torque allowed'
    )

    wrist_estimation = DeclareLaunchArgument(
        'wrist_estimation',
        default_value='False',
        description='Wrist estimation enabling flag'
    )

    contact_threshold = DeclareLaunchArgument(
        'contact_threshold',
        default_value='0.05',
        description='Minimum distance considered between wrist and robot links for force integration'
    )

    no_contact_threshold = DeclareLaunchArgument(
        'no_contact_threshold',
        default_value='0.1',
        description='Maximum distance considered between wrist and robot links for force integration (hysteresis)'
    )

    home_pose = DeclareLaunchArgument(
        'home_pose',
        default_value='[0.6, 0.0, 0.5]',
        description='Home pose for demo node'
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
        prefix = ['taskset -c 4'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    stop_traj_server = Node(
        package='panda_utils',
        executable='stop_traj',
        name='stop_traj_server',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    loop_cart_traj_server = Node(
        package='panda_utils',
        executable='loop_cart_traj',
        name='loop_cart_traj_server',
        prefix = ['taskset -c 5'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    presence_state_node = Node(
        package='panda_utils',
        executable='human_presence',
        name='human_presence',
        prefix = ['taskset -c 3'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wrist_estimation': LaunchConfiguration('wrist_estimation'),
            'contact_threshold': LaunchConfiguration('contact_threshold'),
            'no_contact_threshold': LaunchConfiguration('no_contact_threshold'),
        }],
    )
    frame_publisher = Node(
        package='panda_utils',
        executable='frame_publisher',
        name='frame_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
            'control_freq': LaunchConfiguration('controller_rate'),
            'clamp': LaunchConfiguration('clamp_effort_control'),
            'use_robot': LaunchConfiguration('use_robot'),
            'robot_ip': LaunchConfiguration('robot_ip')

        }],
    )
    impedance_controller = Node(
        package='panda_utils',
        executable='impedance_controller',
        name='impedance_controller',
        # prefix=["gdbserver localhost:3000"],
        # prefix=['taskset -c 10 kitty -e gdb -ex run --args'],
        prefix=['taskset -c 6'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'Kp': LaunchConfiguration('controller_kp'),
            'Kd': LaunchConfiguration('controller_kd'),
            'Md': LaunchConfiguration('controller_md'),
            'Kp_rot': LaunchConfiguration('controller_kp_rot'),
            'Kd_rot': LaunchConfiguration('controller_kd_rot'),
            'Md_rot': LaunchConfiguration('controller_md_rot'),
            'control_freq': LaunchConfiguration('controller_rate'),
            'clamp': LaunchConfiguration('clamp_effort_control'),
            'use_robot': LaunchConfiguration('use_robot'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'world_base_link': LaunchConfiguration('world_base_link'),
            'safe_joint_speed': LaunchConfiguration('safe_joint_speed'),
            'safe_effort_perc': LaunchConfiguration('safe_effort_perc')
        }],
    )
    frame_publisher = Node(
        package='panda_utils',
        executable='frame_publisher',
        name='frame_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    demo = Node(
        package='panda_utils',
        executable='demo',
        name='demo',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'home_pose': LaunchConfiguration('home_pose'),
        }],
    )

    return LaunchDescription([
        use_sim_time,
        controller_kp,
        controller_kd,
        controller_md,
        controller_kp_rot,
        controller_kd_rot,
        controller_md_rot,
        control_rate,
        clamp_effort_control,
        use_robot,
        robot_ip,
        safe_joint_speed,
        safe_effort_perc,
        wrist_estimation,
        contact_threshold,
        no_contact_threshold,
        home_pose,
        world_base_link,
        effort_cmd_server,
        cart_traj_server,
        stop_traj_server,
        loop_cart_traj_server,
        impedance_controller,
        presence_state_node,
        frame_publisher,
        demo,

        inverse_dynamics_controller,
        joint_traj_server,

        # publish_franks_tfs,
        # external_force_estimator,
        # publish_tfs_and_estimate_forces
    ])
