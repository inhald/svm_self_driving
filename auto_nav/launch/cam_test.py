
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

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rs_file_config]),
        launch_arguments={
            'enable_color': 'False',
        }.items()


    )

    ld.add_action(rs_launch)



    return ld