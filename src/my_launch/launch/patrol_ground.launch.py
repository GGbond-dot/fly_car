"""陆空协同搜救 —— 飞车侧全栈入口(Tier0"开始搜救"真跑任务用)。

跟 activity_control_pkg/patrol_test.launch.py 的区别:那个只起桥 + yolo,用来 echo 看
航点到没到,**飞车不会动**;本 launch 起完整的运动链,飞车真跑真飞。

--- 一条航点,地面段和空中段靠 z 分流 ---

  terminal 规划 → UDP FC0A → xmachine_bridge → /wildlife/waypoints(域1 latched)
      → route_target_publisher(pure-pursuit,逐点追) → /target_position
          ├─ chassis_mux 看目标 z:  z ≤ 20cm → /ground_enable → diff_drive_controller → 地面底盘
          │                        z > 20cm → /flight_enable  → pid_controller → 飞控
          ├─ diff_drive_controller(地面时算 v/w)
          └─ pid_controller(空中时算,map 系速度 → uart_to_stm32 按当前 yaw 转机体系 → 串口)

  所以地面链和飞控链**两条都要起** —— 一条路线里既有地面段(z=0)又有空中段(z=120)。
  chassis_mux 负责仲裁,同一时刻只有一条在发速度,不会打架。

--- 两批下发:飞车到起飞点会停住 ---

  第一批(地面段 A1B1→A4B8,全 z=0)跟车一起发 → 飞车开到起飞点 A4B8 停住;
  桥查本地 TF 判到达 → UDP FC0D → terminal 播报"要不要起飞";
  你答"是的" → terminal 发第二批(原地拉高 → 转 yaw 到 -90 → 巡航 → 返航 → 降落)。
  route_target_publisher 收到新路线整条替换,从当前位置接着走。

--- 投放中断(巡航中发现难民) ---

  飞车飞到难民点 → 桥查 TF 判到点 → UDP FC05 → 车 → terminal 播报"要投放物资吗"
      → 你答"需要" → FC06 → 飞车 /terminal_confirm → rescue_drop_sequencer:
          插队降到 drop_z(50cm) → 投货(舵机1) → 插队升回巡航高度 → 接着飞原路线

  "中断"是类比:不打断 route_target_publisher 的队列,而是往它前面 insertNext 插几个
  航点,插的做完原路线自动接着走。走的是跟 YOLO 完全一样的 FC05/FC06 链路 ——
  难民坐标虽然写死,terminal 那边分不出是认出来的还是算出来的。

  ⚠ **难民坐标没定之前 rescuee_check_hz 保持 0** —— 开了 (0,0) 就成难民点,一起飞就误报。

⚠ **绝不能同时起 relief_drop_mission / mission_sequencer** —— 那是拍视频那条写死航点的
   旧剧本,它会自己往 /target_position 发目标,跟规划的路线抢方向盘,飞车会抽。
   而且它也订 /terminal_confirm,会跟 rescue_drop_sequencer 抢同一个确认信号。
   要跑旧剧本就单独 ros2 launch my_launch relief_drop_ground.launch.py,别和本 launch 混。

⚠ **preload_waypoints:=false 必须传** —— 传了 route_test_node 才"静等下发";
   不传就会按它内置的演示航点(前进 2m、升到 100cm 飞方形)自己飞出去。
   (别写成 waypoints:=[] —— 空列表传不进 ROS 2 参数系统,整批节点会起不来。)

--- 用法 ---
    ros2 launch my_launch patrol_ground.launch.py
    ros2 launch my_launch patrol_ground.launch.py with_video:=false   # 不要 YOLO 视频
    ros2 launch my_launch patrol_ground.launch.py launch_check_hz:=0.0  # 关掉到达检测

配对:车侧 car_launch/patrol_test.launch.py(含 terminal)。
自检:export ROS_DOMAIN_ID=1 && python3 fly_car/scripts/test_all.py --where flycar
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SERVO_CAMERA_SCRIPT = os.path.expanduser("~/kian_flycar/scripts/servo_camera_by_mode.py")
DROP_SEQ_SCRIPT = os.path.expanduser("~/kian_flycar/scripts/rescue_drop_sequencer.py")

# 飞行起点 A2B7(场地系,米)。与 terminal 的 planner 必须一致 ——
# planner 现在强制飞车地面直线:起点 (25,-75) → A7 同行中心 (325,-75) cm。
# xmachine_bridge 会再减飞车起点 (25,-75),拿飞车 map 系 TF 判到达,目标即 (300,0)cm。
LAUNCH_TARGET_X_M = 3.25
LAUNCH_TARGET_Y_M = -0.75


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def generate_launch_description():
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py")
    # 地面链:chassis_bridge + diff_drive_controller + chassis_mux(按 z 仲裁地空)
    ground_chassis = _include("ground_chassis_pkg", "ground_chassis.launch.py")
    # 飞控链:串口(含 map→body 速度旋转)+ 位置 PID
    uart = _include("uart_to_stm32", "uart_to_stm32.launch.py")
    position_pid = _include("pid_control_pkg", "position_pid_controller.launch.py")

    # 航点执行器。waypoints 留空 = 不预装,静等 route_topic 下发(见文件头 ⚠)。
    # 容差沿用 ground_step3(拍视频那条跑通过的)。
    route_executor = Node(
        package="activity_control_pkg",
        executable="route_test_node",
        name="route_test_node",
        output="screen",
        parameters=[{
            # ⚠ 别改回 waypoints:=[] —— 空列表传不进 ROS 2 参数系统,launch 会抛
            #   "got '()' of type tuple",整批节点起不来(07-16 上板踩的)。详见 C++ 注释。
            "preload_waypoints": False,             # 不预装:等 terminal 下发
            "route_topic": "/wildlife/waypoints",   # 桥收 FC0A 后转发到这儿
            "position_tolerance_cm": 12.0,
            "lookahead_count": 0,
        }],
    )

    # 跨机 UDP 桥 + 到达飞行起点检测(FC0D)。参数与 patrol_test/relief_drop_mission 一致。
    bridge = Node(
        package="activity_control_pkg",
        executable="xmachine_bridge",
        name="xmachine_bridge",
        output="screen",
        parameters=[{
            # 飞车开机时车头在**场地系**指向哪:0=朝+x(老摆法) / -90=朝-y(现在的摆法)。
            # carto 的 map 系跟着开机姿态走,所以摆法一变,场地系↔map 系就差一个旋转。
            # 桥在这一层统一换算(收航点、难民点/起飞点判定、位置回传),下游全不用管。
            "start_heading_field_deg": LaunchConfiguration("start_heading_field_deg"),
            "field_origin_x_m": 0.25,   # 飞车起点场地系 (25,-75)cm,场地→飞车map先减它
            "field_origin_y_m": -0.75,
            "car_ip": "192.168.10.161",
            "to_car_port": 8890,
            "from_car_port": 8891,
            "send_hz": 10.0,
            "resend_count": 30,
            "pose_hz": 1.0,             # 飞车位置回传(FC0C),/slam 页点云靠它逐步露出
            # 到达飞行起点判定:三道关(TF 新鲜 + 进容差 + 稳住)全过才发 FC0D,只发一次
            "launch_target_x_m": LAUNCH_TARGET_X_M,
            "launch_target_y_m": LAUNCH_TARGET_Y_M,
            "launch_tol_m": 0.15,
            "launch_stable_s": 0.5,
            "launch_tf_fresh_s": 0.5,
            "launch_check_hz": LaunchConfiguration("launch_check_hz"),
            # 难民点:飞到这儿发 FC05 → terminal 播报"要投放物资吗"。走的是跟 YOLO
            # 一模一样的链路,terminal 分不出是认出来的还是算出来的。
            # **坐标待实测,所以 rescuee_check_hz 默认 0(关掉)** —— 给了坐标再打开,
            # 否则 (0,0) 会被当成难民点,飞车一起飞就报"识别到难民"。
            "rescuee_x_m": LaunchConfiguration("rescuee_x_m"),
            "rescuee_y_m": LaunchConfiguration("rescuee_y_m"),
            "rescuee_tol_m": 0.25,
            "rescuee_check_hz": LaunchConfiguration("rescuee_check_hz"),
            # 这个难民是**飞车投的空中难民** → class=1。terminal 据此把"确认投放"发
            # /terminal_confirm(→FC06→飞车 rescue_drop),而不是 /car/drop_confirm(车投)。
            # 默认是 2(地面难民=车投),不改的话飞车永远收不到确认、不投货。
            "rescuee_point_class": 1,
        }],
    )

    # 摄像头舵机跟着地空模式自动切:地面 120°,起飞后 180°(垂直俯视,YOLO 认难民靠它)。
    # 拍视频那套每条 launch 只管一个阶段,servo_set_once 摆一次就行;本任务一条路线
    # 横跨地面段和空中段,中途必须换 —— 跟 chassis_mux 的 /flight_enable 走。
    camera = ExecuteProcess(
        cmd=["python3", SERVO_CAMERA_SCRIPT,
             "--index", "2", "--ground-deg", "120", "--air-deg", "180"],
        output="screen",
    )

    # 投放中断:收到 terminal 的"确认投放"(FC06 → /terminal_confirm)就
    # 降到 drop_z → 投货 → 升回原高度 → 接着飞原巡航路线(靠 insertNext 插队,不打断队列)。
    # ⚠ 旧剧本 mission_sequencer 也订 /terminal_confirm,所以那个绝不能跟本 launch 同时起。
    drop_sequencer = ExecuteProcess(
        cmd=["python3", DROP_SEQ_SCRIPT,
             "--drop-z", LaunchConfiguration("drop_z_cm"),
             "--servo-index", "1", "--open-deg", "180", "--close-deg", "90",
             "--t-drop-s", "1.5"],
        output="screen",
    )

    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("yolo_detector_pkg"),
            "launch", "yolo_detector.launch.py")),
        condition=IfCondition(LaunchConfiguration("with_video")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "with_video", default_value="true",
            description="起 yolo_detector,画面经 UDP FC08 发给车的 /slam 页「机」那一路",
        ),
        DeclareLaunchArgument(
            "launch_check_hz", default_value="10.0",
            description="到达飞行起点检测频率(Hz)。0 = 关掉,飞车到 A4B8 不会回传 FC0D",
        ),
        # 2026-07-16 把飞车摆成开机就朝场地 -y —— 地面段第一个航点是 (0,0)yaw=-90,
        # 摆成朝 +x 的话起步先要原地转 90°(移动和转 yaw 不能同时,那一转纯浪费)。
        # 摆成 -y 后换算下来第一个航点 yaw=0 = 开机姿态,那个原地转就没了。
        # ⚠ 改回老摆法(车头朝 +x)就把它设回 0.0,不用重编译。
        DeclareLaunchArgument(
            "start_heading_field_deg", default_value="0.0",
            description="飞车开机时车头在场地系的朝向(度):0=朝+x / -90=朝-y。"
                        "桥按它换算场地系↔map 系",
        ),
        # 难民点 (0.5,-3.0)m = A7B2。已核对**飞车真飞得到**:在右区、不在箱子不遍历带、
        # 且正好落在巡航航点 (50,-250)→(50,-400) 这条竖直直线段上,飞车从它正上方过。
        # (它不是航点本身,而是直线段中间的点 —— 无所谓,触发靠查 TF 位置,跟是不是航点无关。)
        #
        # ⚠ 曾经写的 (0.5,-2.0)=A5B2 是**飞不到的**:那格是箱子不遍历带,规划时被排掉,
        #   路线根本不经过,到点永远不触发。改坐标前先跑一遍可达性核对。
        DeclareLaunchArgument(
            "rescuee_x_m", default_value="0.66", description="难民2 X(场地系,米)"),
        DeclareLaunchArgument(
            "rescuee_y_m", default_value="-3.13", description="难民2 Y(场地系,米)"),
        DeclareLaunchArgument(
            "rescuee_check_hz", default_value="10.0",
            description="到难民点检测频率(Hz)。0 = 关掉,飞到难民点也不会播报",
        ),
        DeclareLaunchArgument(
            "drop_z_cm", default_value="50.0", description="投放高度(cm),从巡航高度降到这儿投"),
        # 飞车整条链路进域 1,与车板(域 0)隔离。跨机走裸 UDP,与域无关。
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
        fly_carto,
        # 桥不依赖 TF 就能收发,早点起来别漏了车发的航点。
        # (到达检测拿不到 TF 时自己会 warn 并暂停,不会崩)
        bridge,
        yolo,
        # 等雷达/carto 出 TF 再上运动链。carto 现在 T+5s 起(见 fly_carto),留 3s 出 TF,
        # 8s 上运动链 —— 比原来 12s 快 4s(飞车启动比车慢的主因)。route_target/pid 早起
        # 拿不到 TF 只会空转 warn、不崩,所以缩短安全。还想更快可再降,但别早于 carto 出 TF。
        TimerAction(
            period=8.0,
            actions=[ground_chassis, uart, position_pid, route_executor,
                     camera, drop_sequencer],
        ),
    ])
