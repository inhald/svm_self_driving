
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    ld = LaunchDescription()

    rs_file_config = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )


    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )

    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )


    rplidar_config = os.path.join(
        get_package_share_directory('auto_nav'),
        'config',
        'sllidar.yaml'
    )

    nav_config = os.path.join(
        get_package_share_directory('auto_nav'),
        'config',
        'params.yaml'

    )

    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )

    exp_config = os.path.join(
        get_package_share_directory('auto_nav'),
        'config',
        'experiment.yaml'

    )


    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')




    rplidar_la = DeclareLaunchArgument(
        'rplidar_config',
        default_value = rplidar_config,
        description='launch args for sllidar'

    )


    nav_la = DeclareLaunchArgument(
        'nav_config',
        default_value = nav_config,
        description='launch args for autonomous navigation'

    )

    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    

    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')

    exp_la = DeclareLaunchArgument(
        'exp_config',
        default_value = exp_config,
        description ='launch args'

    )

    ld = LaunchDescription([joy_la, rplidar_la, nav_la, vesc_la, mux_la, exp_la])



    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )

    rplidar_node = Node (
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[LaunchConfiguration('rplidar_config')],
        remappings=[('/scan', '/scan2')]
    )

    nav_node = Node(
        package='auto_nav',
        executable='navigation',
        name='navigation',
        parameters=[LaunchConfiguration('nav_config')]
    )

    static_tf_node1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.1286', '0.0', '0.076', '0.0', '0.0', '1.0', 'base_link', 'laser']
    )

    static_tf_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_camera',
        arguments=['0.292', '0.0', '0.86', '-1.5708', '0.0', '-1.5708', 'base_link', 'laser']
    )

    experiment_node = Node(
        package='auto_nav',
        executable='experiment',
        name = 'experiment',
        parameters=[LaunchConfiguration('exp_config')]
    )


    #vesc nodes
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )


    #Mux
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )


    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
    )


    # rs_camera_node = Node(
    #     package='realsense2_camera',
    #     executable='realsense2_camera_node',
    #     parameters=[
    #         {'enable_color': False},
    #         {'enable_depth': True},

    #     ]

    # )



    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rs_file_config]),
        launch_arguments={
            'enable_color': 'False',
            'enable_depth': 'True',
            'initial_reset': 'True',
            'accelerate_gpu_with_glsl': 'True',
            

            
        }.items()


    )

    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(rs_launch)
    ld.add_action(rplidar_node)
    ld.add_action(static_tf_node1)
    ld.add_action(static_tf_node2)
    ld.add_action(experiment_node)
    # ld.add_action(throttle_interpolator_node)

    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(nav_node)

    return ld