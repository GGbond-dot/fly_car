import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, LifecycleNode
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart, OnStateTransition

def generate_launch_description():
    # 1. 获取包路径
    my_carto_pkg_share = FindPackageShare(package='my_carto_pkg').find('my_carto_pkg')
    uart_to_stm32_pkg_share = FindPackageShare(package='uart_to_stm32').find('uart_to_stm32')
    pid_control_pkg_share = FindPackageShare(package='pid_control_pkg').find('pid_control_pkg')
    activity_control_pkg_share = FindPackageShare(package='activity_control_pkg').find('activity_control_pkg')
    
    # 2. 定义 Launch 文件包含
    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_carto_pkg_share, 'launch', 'fly_carto.launch.py')
        )       
    )
    
    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_to_stm32_pkg_share, 'launch', 'uart_to_stm32.launch.py')
        )
    )
    
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_control_pkg_share, 'launch', 'position_pid_controller.launch.py')
        )
    )   

    route_test_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(activity_control_pkg_share, 'launch', 'route_test.launch.py')
        )
    )
    
    # 3. 定义节点
    # [关键] 使用 LifecycleNode 以支持状态管理
    control_node_lifecycle = LifecycleNode(
        package='pid_controller',
        executable='control_node_lifecycle',
        name='control_node_lifecycle',
        namespace='',
        output='screen',
    )
    
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
    
    openmv_bridge = Node(
        package='openmv_bridge',
        executable='openmv_bridge',
        name='openmv_bridge',
        output='screen'
    )
    
    bluetooth_node = Node(
        package='bluetooth',
        executable='bluetooth_node',
        name='bluetooth_node',
        output='screen'
    )

    # 4. 定义生命周期事件
    # 定义 Configure 请求
    configure_request = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(control_node_lifecycle),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # 定义 Activate 请求
    active_request = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(control_node_lifecycle),
            transition_id=Transition.TRANSITION_ACTIVATE
        )
    )
    
    # 5. 定义事件处理器
    # 逻辑：节点启动 -> 等待 2 秒 -> 发送 Configure
    configure_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node_lifecycle,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[configure_request]
                )
            ]
        )
    )

    # 逻辑：节点状态变为 inactive (即 Configure 完成) -> 发送 Activate
    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=control_node_lifecycle,
            start_state='unconfigured',
            goal_state='inactive',
            entities=[active_request]
        )
    )

    # 6. 定义延时启动组 (3秒后启动)
    delayed_group = TimerAction(
        period=3.0,
        actions=[
            # 先注册处理器，防止错过事件
            configure_handler,
            activate_handler,
            
            # 启动其他节点
            uart_to_stm32_launch,
            position_pid_controller_launch,
            route_test_launch,
            control_node_lifecycle,
            car_driver_node,
            openmv_bridge,
            bluetooth_node,
        ]
    )

    return LaunchDescription([
        fly_carto_launch, # 立即启动
        delayed_group     # 3秒后启动其余部分
    ])
# import os
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.actions import Node
# from launch.events import matches_action
# from launch_ros.actions import LifecycleNode
# from launch_ros.events.lifecycle import ChangeState
# from lifecycle_msgs.msg import Transition
# from launch.event_handler import OnProcessStart, OnStateTransition
# from launch.actions import RegisterEventHandler, TimerAction, EmitEvent

# def generate_launch_description():
#     my_carto_pkg_share = FindPackageShare(package='my_carto_pkg').find('my_carto_pkg')
#     uart_to_stm32_pkg_share = FindPackageShare(package='uart_to_stm32').find('uart_to_stm32')
#     pid_control_pkg_share = FindPackageShare(package='pid_control_pkg').find('pid_control_pkg')
#     activity_control_pkg_share = FindPackageShare(package='activity_control_pkg').find('activity_control_pkg')
#     # car_driver_share = FindPackageShare(package='car_driver').find('car_driver')
#     fly_carto_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(my_carto_pkg_share, 'launch', 'fly_carto.launch.py')
#         )       
#     )
    
#     uart_to_stm32_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(uart_to_stm32_pkg_share, 'launch', 'uart_to_stm32.launch.py')
#         )
#     )
    
#     position_pid_controller_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pid_control_pkg_share, 'launch', 'position_pid_controller.launch.py')
#         )
#     )   

#     route_test_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(activity_control_pkg_share, 'launch', 'route_test.launch.py')
#         )
#     )
    
#     control_node_lifecycle = Node(
#         package='pid_controller',
#         executable='control_node_lifecycle',
#         name='control_node_lifecycle',
#         output='screen',
#     )
    
#     car_driver_node = Node(
#         package='car_driver',
#         executable='car_driver',
#         name='car_driver',
#         output='screen',
#         parameters=[
#             {'serial_port': '/dev/ttyUSB1'},
#             {'baudrate': 115200},
#             {'motor_type': 2},
#         ]
#     )
    
#     openmv_bridge = Node(
#         package='openmv_bridge',
#         executable='openmv_bridge',
#         name='openmv_bridge',
#         output='screen'
#     )
    
#     bluetooth_node = Node(
#         package='bluetooth',
#         executable='bluetooth_node',
#         name='bluetooth_node',
#         output='screen'
#     )

#     configure_request = EmitEvent(
#         event=ChangeState(
#             lifecycle_node_matcher = control_node_lifecycle,
#             transition_id=Transition.TRANSITION_CONFIGURE
#         )
#     )

#     active_request = EmitEvent(
#         event=ChangeState(
#             lifecycle_node_matcher = matches_action(control_node_lifecycle),
#             transition_id=Transition.TRANSITION_ACTIVATE
#         )
#     )
    
#     configure_handler = RegisterEventHandler(
#         OnProcessStart(
#             target_action=control_node_lifecycle,
#             on_start=[
#                 TimerAction(
#                     period=2.0,
#                     actions=[configure_request]
#                 )
#             ]
#         )
#     )

#     activate_handler = RegisterEventHandler(
#         OnStateTransition(
#             target_lifecycle_node=control_node_lifecycle,
#             start_state='unconfigured',
#             goal_state='inactive',
#             entities=[active_request]
#         )
#     )

#     delayed_group = TimerAction(
#         period=3.0,
#         actions=[
#             uart_to_stm32_launch,
#             position_pid_controller_launch,
#             route_test_launch,
#             control_node_lifecycle,
#             car_driver_node,
#             openmv_bridge,
#             bluetooth_node,
#             configure_handler,
#             activate_handler
#         ]
#     )

#     # delayed_configure_event = TimerAction(
#     #     period=5.0,
#     #     actions=[
#     #         EmitEvent(
#     #             event=ChangeState(
#     #                 lifecycle_node=car_driver_node,
#     #                 transition=Transition.TRANSITION_CONFIGURE
#     #             )
#     #         )
#     #     ]
#     # )
    
#     # activate_event_handler = RegisterEventHandler(
#     #     event_handler=launch.event_handlers.OnStateTransition(
#     #         target_lifecycle_node=car_driver_node,
#     #         start_state='inactive',
#     #         goal_state='active',
#     #         entities=[
#     #             EmitEvent(
#     #                 event=ChangeState(
#     #                     lifecycle_node=car_driver_node,
#     #                     transition=Transition.TRANSITION_ACTIVATE
#     #                 )
#     #             )
#     #         ]
#     #     )   
#     # )

#     # car_driver_launch = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource(
#     #         os.path.join(car_driver_share, 'launch', 'car_drive.launch.py')
#     #     )
#     # )
#     return LaunchDescription([
#         fly_carto_launch,
#         delayed_group
#     ])