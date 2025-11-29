from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    car_driver_node = Node(
        package='car_driver',
        executable='car_driver',
        name='car_driver',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyUSB1'},
            {'baudrate': 115200},
            {'motor_type': 2},
        ]
    )
    
    wheel_speeds_pub_node = Node(
        package='car_driver',
        executable='wheel_speeds_pub',
        name='wheel_speeds_pub',
        output='screen',    
    )

    return LaunchDescription([
        car_driver_node,
        wheel_speeds_pub_node
    ])