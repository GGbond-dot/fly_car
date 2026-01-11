# ROS2 常用命令速查表 (Quick Reference)

## 节点 (Node) 相关命令

### 查看节点
```bash
# 列出所有活动节点
ros2 node list

# 查看节点详细信息
ros2 node info /node_name

# 查看节点的发布/订阅话题
ros2 node info /node_name
```

### 运行节点
```bash
# 运行单个节点
ros2 run <package_name> <executable_name>

# 使用启动文件
ros2 launch <package_name> <launch_file>
```

## 话题 (Topic) 相关命令

### 查看话题
```bash
# 列出所有话题
ros2 topic list

# 查看话题类型
ros2 topic type /topic_name

# 查看话题详细信息
ros2 topic info /topic_name

# 显示话题消息结构
ros2 interface show <message_type>
```

### 监听和发布话题
```bash
# 监听话题内容
ros2 topic echo /topic_name

# 查看话题发布频率
ros2 topic hz /topic_name

# 查看话题带宽
ros2 topic bw /topic_name

# 发布话题数据
ros2 topic pub /topic_name <message_type> '{data: value}'

# 示例：发布速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```

## TF 变换相关命令

### 查看坐标变换
```bash
# 查看所有 TF 变换
ros2 topic echo /tf

# 生成 TF 树形图
ros2 run tf2_tools view_frames

# 查看两个坐标系之间的变换
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>
```

## 参数 (Parameter) 相关命令

### 查看和设置参数
```bash
# 列出所有参数
ros2 param list

# 获取参数值
ros2 param get /node_name parameter_name

# 设置参数值
ros2 param set /node_name parameter_name value

# 导出节点参数到文件
ros2 param dump /node_name

# 从文件加载参数
ros2 param load /node_name params.yaml
```

## 服务 (Service) 相关命令

### 查看和调用服务
```bash
# 列出所有服务
ros2 service list

# 查看服务类型
ros2 service type /service_name

# 查看服务详细信息
ros2 service info /service_name

# 调用服务
ros2 service call /service_name <service_type> '{request: value}'
```

## 包 (Package) 相关命令

### 包管理
```bash
# 列出所有包
ros2 pkg list

# 查找包的路径
ros2 pkg prefix <package_name>

# 查看包的可执行文件
ros2 pkg executables <package_name>

# 创建新包
ros2 pkg create --build-type ament_cmake <package_name>
ros2 pkg create --build-type ament_python <package_name>
```

## 编译和构建

### Colcon 构建命令
```bash
# 编译整个工作空间
colcon build

# 编译特定包
colcon build --packages-select <package_name>

# 编译并显示详细输出
colcon build --event-handlers console_direct+

# 仅编译某个包及其依赖
colcon build --packages-up-to <package_name>

# 清理构建文件
rm -rf build/ install/ log/
```

### 设置环境
```bash
# Source ROS2 环境
source /opt/ros/<distro>/setup.bash

# Source 工作空间
source install/setup.bash

# 同时 source (在 .bashrc 中添加)
source /opt/ros/humble/setup.bash
source ~/workspace/install/setup.bash
```

## 调试工具

### RQT 工具
```bash
# 打开 RQT 图形界面
rqt

# 话题监控图
rqt_graph

# 数据绘图
rqt_plot /topic_name/field

# 控制台
rqt_console

# 参数配置
rqt_reconfigure
```

### 日志查看
```bash
# 设置日志级别
ros2 run <package> <node> --ros-args --log-level debug

# 日志级别: DEBUG, INFO, WARN, ERROR, FATAL
```

## 常用调试场景

### 场景 1: 节点无法通信
```bash
# 1. 检查节点是否运行
ros2 node list

# 2. 检查话题是否存在
ros2 topic list

# 3. 检查话题类型是否匹配
ros2 topic type /topic_name

# 4. 监听话题是否有数据
ros2 topic echo /topic_name

# 5. 检查 QoS 设置
ros2 topic info /topic_name
```

### 场景 2: TF 变换查询失败
```bash
# 1. 检查 TF 树
ros2 run tf2_tools view_frames

# 2. 监听 /tf 话题
ros2 topic echo /tf

# 3. 检查特定变换
ros2 run tf2_ros tf2_echo source_frame target_frame

# 4. 检查 TF 发布频率
ros2 topic hz /tf
```

### 场景 3: 串口连接问题
```bash
# 查看串口设备
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*

# 检查串口权限
sudo chmod 666 /dev/ttyUSB0

# 将用户添加到 dialout 组
sudo usermod -aG dialout $USER

# 测试串口通信
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 921600
```

### 场景 4: 参数调试
```bash
# 查看当前参数
ros2 param list

# 动态修改 PID 参数
ros2 param set /pid_controller pid_x_kp 0.5
ros2 param set /pid_controller pid_x_ki 0.1
ros2 param set /pid_controller pid_x_kd 0.05

# 保存参数配置
ros2 param dump /pid_controller > pid_params.yaml
```

## Fly Car 项目特定命令

### 启动系统
```bash
# 启动完整系统
ros2 launch uart_to_stm32 total.launch.py

# 启动 PID 控制器
ros2 launch pid_control_pkg position_pid_controller.launch.py

# 启动路径跟踪
ros2 launch activity_control_pkg route_target_publisher.launch.py

# 启动 SLAM
ros2 launch my_carto_pkg fly_carto.launch.py
```

### 监控关键话题
```bash
# 监控目标位置
ros2 topic echo /target_position

# 监控目标速度
ros2 topic echo /target_velocity

# 监控高度
ros2 topic echo /height

# 监控 STM32 就绪状态
ros2 topic echo /is_st_ready

# 监控速度命令
ros2 topic echo /velocity_map
```

### 手动发布测试数据
```bash
# 发布目标位置 (x=100cm, y=50cm, yaw=45deg)
ros2 topic pub /target_position std_msgs/msg/Float32MultiArray \
  '{data: [100.0, 50.0, 45.0]}'

# 发布速度命令
ros2 topic pub /velocity_map geometry_msgs/msg/Twist \
  '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'
```

### 查看系统状态
```bash
# 查看 TF 变换
ros2 run tf2_ros tf2_echo map laser_link

# 查看 PID 参数
ros2 param list | grep pid

# 查看所有活动节点
ros2 node list

# 查看节点连接图
rqt_graph
```

## 实用技巧

### 命令别名 (添加到 ~/.bashrc)
```bash
# ROS2 常用别名
alias rt='ros2 topic'
alias rn='ros2 node'
alias rp='ros2 param'
alias rs='ros2 service'
alias cb='colcon build'
alias cbs='colcon build --symlink-install'
alias src='source install/setup.bash'

# Fly Car 特定别名
alias fly_start='ros2 launch uart_to_stm32 total.launch.py'
alias fly_pid='ros2 launch pid_control_pkg position_pid_controller.launch.py'
```

### 查看帮助
```bash
# 查看命令帮助
ros2 --help
ros2 topic --help
ros2 node --help

# 查看子命令帮助
ros2 topic echo --help
ros2 param set --help
```

## 考试重点命令

| 命令 | 功能 | 重要性 |
| ---- | ---- | ------ |
| `ros2 node list` | 查看节点 | ⭐⭐⭐⭐⭐ |
| `ros2 topic echo` | 监听话题 | ⭐⭐⭐⭐⭐ |
| `ros2 topic list` | 查看话题 | ⭐⭐⭐⭐⭐ |
| `ros2 param set` | 设置参数 | ⭐⭐⭐⭐⭐ |
| `ros2 run tf2_ros tf2_echo` | 查看 TF | ⭐⭐⭐⭐ |
| `rqt_graph` | 可视化节点连接 | ⭐⭐⭐⭐ |
| `colcon build` | 编译项目 | ⭐⭐⭐⭐⭐ |
| `ros2 topic pub` | 发布话题 | ⭐⭐⭐⭐ |

---

**提示**: 考试时可以用 `--help` 查看命令帮助！
