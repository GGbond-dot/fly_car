"""飞车连接车端开放热点(STA/客户端)。与车端 wifi_ap_manager 的 open_ap.launch.py 对称。

车端先 `ros2 launch wifi_ap_manager open_ap.launch.py`(action:=start)开热点,
飞车端再跑本 launch 连入。默认静态 IP 192.168.50.2,网关指向车 192.168.50.1。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("action", default_value="connect"),
            DeclareLaunchArgument("ssid", default_value="OPi_ROS2_TEST"),
            DeclareLaunchArgument("interface", default_value="wlan0"),
            DeclareLaunchArgument("connection_name", default_value="OPi_ROS2_JOIN_AP"),
            DeclareLaunchArgument("ipv4_method", default_value="manual"),
            DeclareLaunchArgument("ip_cidr", default_value="192.168.50.2/24"),
            DeclareLaunchArgument("gateway", default_value="192.168.50.1"),
            DeclareLaunchArgument("rescan", default_value="true"),
            Node(
                package="wifi_sta_manager",
                executable="sta_manager",
                name="wifi_sta_manager",
                output="screen",
                parameters=[
                    {
                        "action": ParameterValue(LaunchConfiguration("action"), value_type=str),
                        "ssid": ParameterValue(LaunchConfiguration("ssid"), value_type=str),
                        "interface": ParameterValue(LaunchConfiguration("interface"), value_type=str),
                        "connection_name": ParameterValue(
                            LaunchConfiguration("connection_name"), value_type=str
                        ),
                        "ipv4_method": ParameterValue(LaunchConfiguration("ipv4_method"), value_type=str),
                        "ip_cidr": ParameterValue(LaunchConfiguration("ip_cidr"), value_type=str),
                        "gateway": ParameterValue(LaunchConfiguration("gateway"), value_type=str),
                        "rescan": ParameterValue(LaunchConfiguration("rescan"), value_type=bool),
                    }
                ],
            ),
        ]
    )
