# Usage:
# 	- connection through Skycontroller [recommended]:
# 		ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi' ip:='192.168.53.1'
# 	- direct connection to Anafi:
# 		ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi' ip:='192.168.42.1' model:='ai'
# 	- connection to the simulated drone in Sphinx:
# 		ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi' ip:='10.202.0.1' model:='ai'

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # args that can be set from the command line or default will be used

    ip_arg = DeclareLaunchArgument(
        "ip",
        default_value="192.168.53.1",  # Anafi: '192.168.42.1', SkyController: '192.168.53.1', Sphinx: '10.202.0.1'
        description="IP address of the device",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="anafi",
        description="Namespace of the drone",
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="4k",  # {'4k', 'thermal', 'usa', 'ai'}
        description="Model of the drone",
    )
    lab_arg = DeclareLaunchArgument(
        "lab",
        default_value="False",
        description="True if the drone is in the lab",
    )
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="True",
        description="True if the drone is simulated",
    )

    config = os.path.join(get_package_share_directory("anafi_ros_nodes"), "params.yaml")

    anafi_node = Node(
        package="anafi_ros_nodes",
        namespace=LaunchConfiguration("namespace"),
        executable="anafi",
        name="anafi",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--params-file", config, "--log-level", "INFO"],
        parameters=[
            {"drone/model": LaunchConfiguration("model")},
            {"device/ip": LaunchConfiguration("ip")},
        ],
    )

    interface_node = Node(
        package="anafi_ros_nodes",
        namespace=LaunchConfiguration("namespace"),
        executable="anafi_interface",
        name="anafi_interface",
        output="screen",
        parameters=[
            {"qualisys/available": LaunchConfiguration("lab") == "True"},
            {"simulator/enabled": LaunchConfiguration("sim") == "True"},
            {"cmd_scale/roll": 1.0},
            {"cmd_scale/pitch": 1.0},
            {"cmd_scale/thrust": 1.0},
        ],
    )

    qualisys_downsampler_node = Node(
        package="topic_tools",
        executable="throttle",
        name="qualisys_downsampler",
        output="screen",
        arguments=[
            "messages",
            "/qualisys/Anafi/pose",
            "20",
            "/qualisys/Anafi/pose_downsampled",
        ],  # Downsampled to simulate GNSS frequency
    )

    return LaunchDescription(
        [
            namespace_arg,
            ip_arg,
            model_arg,
            lab_arg,
            sim_arg,
            anafi_node,
            interface_node,
            qualisys_downsampler_node,
        ]
    )
