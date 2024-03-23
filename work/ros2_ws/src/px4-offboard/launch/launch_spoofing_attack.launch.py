import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # param Bridge
    param_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/AbuDhabi/set_pose@ros_gz_interfaces/srv/SetEntityPose']
        
    )

    spoofer_node = Node(
        package='px4_offboard',
        executable='spoofer_gz_stream',
        name="spoofer",
        output = 'screen'     

    )

    return LaunchDescription([
        param_bridge,
        spoofer_node,
    ])