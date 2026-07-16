#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>
#include <clocale>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

namespace
{
constexpr double kDefaultTimerPeriodSec = 0.05;
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_idx_(std::numeric_limits<std::size_t>::max()),
  has_height_(false),
  current_height_cm_(0.0)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  ground_z_tol_cm_ = declare_parameter("ground_z_tol_cm", 30.0);  // 地面:松(大)
  // 地面航点只按 x(行进方向)判到达,不看 y/yaw。飞车锁 yaw=0 直行修不了横向,
  // y 会漂 30cm 导致按 xy 判永远到不了、冲过头(见 isReached 注释)。默认开。
  ground_reach_x_only_ = declare_parameter("ground_reach_x_only", true);
  // 起飞时把“起飞点正上方”航点的 xy 换成飞车当前 TF 位置(就地拉高,不横移)。默认开。
  takeoff_use_current_xy_ = declare_parameter("takeoff_use_current_xy", true);
  air_z_tol_cm_ = declare_parameter("air_z_tol_cm", 8.0);         // 空中:紧(小)
  land_z_cm_ = declare_parameter("land_z_cm", 4.0);              // land_after 落点高度
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");
  // terminal 现场规划的路线从这个话题来(xmachine_bridge 收 UDP FC0A 后本地转发)。
  // 设成空字符串 = 不订阅,只认启动参数 —— 拍视频那套写死航点的 launch 就该这么配。
  route_topic_ = declare_parameter<std::string>("route_topic", "/wildlife/waypoints");
  // 投放中断插队航点(降 50 → 升回)。跟 route_topic 不同:它不清队列,做完接着原路线。
  insert_topic_ = declare_parameter<std::string>("insert_topic", "/route/insert_waypoints");
  // pure-pursuit 前视:>0 时在 /target_position 消息后追加接下来 N 个航点的 xy(cm),供控制器取前视点。
  // 默认 0=不追加,消息仍是 [x,y,z,yaw] 4 位,老订阅者(chassis_mux 等)完全兼容。
  lookahead_count_ = static_cast<std::size_t>(declare_parameter("lookahead_count", 0));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, qos);

  is_st_ready_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/is_st_ready", 10, std::bind(&RouteTargetPublisherNode::is_st_ready_callback, this, std::placeholders::_1));

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));

  // 桥那边是 latched 发的(晚起也能拿到最后一条路线),这里 QoS 要对上,否则收不到。
  // 重复包在桥里已按 wp_id 去重过,到这儿的每条都是新路线。
  if (!route_topic_.empty()) {
    route_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      route_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&RouteTargetPublisherNode::routeCallback, this, std::placeholders::_1));
  }
  // 插队用 volatile:它是"此刻插一下"的一次性事件,latched 会让节点重启后又插一次。
  if (!insert_topic_.empty()) {
    insert_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      insert_topic_, rclcpp::QoS(10),
      std::bind(&RouteTargetPublisherNode::insertCallback, this, std::placeholders::_1));
  }

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  RCLCPP_INFO(get_logger(),
    "RouteTargetPublisher initialized: map=%s laser_link=%s topic=%s", map_frame_.c_str(),
    laser_link_frame_.c_str(), output_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm",
    pos_tol_cm_, yaw_tol_deg_, height_tol_cm_);
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  targets_.push_back(target);
  if (was_empty) {
    current_idx_ = 0;
    publishCurrent();
  }
}

void RouteTargetPublisherNode::setRoute(const std::vector<Target> & batch_in)
{
  if (batch_in.empty()) {
    RCLCPP_WARN(get_logger(), "收到空路线,忽略(不清空当前队列)");
    return;
  }
  std::vector<Target> batch = batch_in;

  // 起飞用**当前真实位置**就地拉高:空中段(首点 z>20)时,把开头“起飞点正上方”那几个
  // 航点(xy 与首点相同 = ①原地拉高 ②转 yaw)的 xy 换成飞车当前 TF 的 xy。
  // 因为地面段只按 x 判到达,飞车实际可能停在偏了 y 30cm 的地方;不换的话飞车会先横移
  // 回规划的起飞点再拉高。换成当前 xy = 我在哪就从哪原地拔高。后续遍历航点不动。
  if (takeoff_use_current_xy_ && batch.front().z_cm > 20.0) {
    double cx = 0.0, cy = 0.0, cz = 0.0, cyaw = 0.0;
    if (getCurrentPose(cx, cy, cz, cyaw)) {
      const double fx = batch.front().x_cm;
      const double fy = batch.front().y_cm;
      bool first_takeoff = true;
      for (auto & t : batch) {
        if (std::hypot(t.x_cm - fx, t.y_cm - fy) < 1.0) {  // 与起飞点同 xy = 起飞拉高段
          t.x_cm = cx;
          t.y_cm = cy;
          if (first_takeoff) {
            // ①原地拉高:xy 和 yaw 都用停下时的真实姿态,拉高**全程一点都不动 yaw**,
            // 免得边爬升边拧机头出问题。②转 yaw 点(下一个)才转到规划的飞行航向。
            t.yaw_deg = cyaw;
          }
          first_takeoff = false;
        } else {
          break;  // 到第一个遍历点就停,遍历航点保持规划的绝对坐标
        }
      }
      RCLCPP_INFO(get_logger(),
        "起飞:用当前姿态 (%.0f,%.0f,yaw=%.0f) 就地拉高,拉高不动 yaw,不横移", cx, cy, cyaw);
    } else {
      RCLCPP_WARN(get_logger(), "起飞时拿不到 TF,退回用规划起飞点 xy");
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  targets_ = batch;
  current_idx_ = 0;
  RCLCPP_INFO(get_logger(), "整条换路线: %zu 个航点,从头开始追", targets_.size());
  publishCurrent();
}

namespace
{
// [x,y,z,yaw,...] 扁平数组 → Target 序列。长度不合法返回空。
std::vector<Target> parseWaypointArray(const std::vector<float> & d)
{
  std::vector<Target> batch;
  if (d.size() < 4 || d.size() % 4 != 0) {
    return batch;
  }
  batch.reserve(d.size() / 4);
  for (std::size_t i = 0; i + 3 < d.size(); i += 4) {
    batch.push_back(Target{d[i], d[i + 1], d[i + 2], d[i + 3]});
  }
  return batch;
}
}  // namespace

void RouteTargetPublisherNode::routeCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  const auto batch = parseWaypointArray(msg->data);
  if (batch.empty()) {
    RCLCPP_ERROR(get_logger(),
      "路线必须是 4 的倍数 [x_cm,y_cm,z_cm,yaw_deg,...],收到 %zu 个数,忽略", msg->data.size());
    return;
  }
  RCLCPP_INFO(get_logger(), "收到 %s 的新路线(%zu 航点),首点 x=%.1f y=%.1f z=%.1f yaw=%.1f",
    route_topic_.c_str(), batch.size(),
    batch.front().x_cm, batch.front().y_cm, batch.front().z_cm, batch.front().yaw_deg);
  setRoute(batch);
}

// 投放中断:插队航点(降 50 → 升回)。做完自动接着原巡航路线,**不清队列** ——
// 跟 route_topic 的"整条换掉"完全不同,别搞混。
void RouteTargetPublisherNode::insertCallback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  const auto batch = parseWaypointArray(msg->data);
  if (batch.empty()) {
    RCLCPP_ERROR(get_logger(),
      "插队航点必须是 4 的倍数,收到 %zu 个数,忽略", msg->data.size());
    return;
  }
  RCLCPP_INFO(get_logger(), "插队 %zu 个航点(首点 x=%.1f y=%.1f z=%.1f),做完接着走原路线",
    batch.size(), batch.front().x_cm, batch.front().y_cm, batch.front().z_cm);
  insertNext(batch);
}

void RouteTargetPublisherNode::insertNext(const std::vector<Target> & batch)
{
  if (batch.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // 队列还没开始(无当前目标):退化为依次追加
  if (current_idx_ == std::numeric_limits<std::size_t>::max() || targets_.empty()) {
    const bool was_empty = targets_.empty();
    targets_.insert(targets_.end(), batch.begin(), batch.end());
    if (was_empty) {
      current_idx_ = 0;
      publishCurrent();
    }
    return;
  }

  // 所有目标已完成(current_idx_ 越界)时,从队尾接着追加并重启推进
  if (current_idx_ >= targets_.size()) {
    current_idx_ = targets_.size();
    targets_.insert(targets_.end(), batch.begin(), batch.end());
    publishCurrent();
    return;
  }

  // 正常情况:在当前目标之前插入 batch。current_idx_ 数值不变,
  // 但现在指向 batch 的第一个(起飞点);原当前目标顺延到 batch 之后。
  targets_.insert(targets_.begin() + static_cast<std::ptrdiff_t>(current_idx_),
    batch.begin(), batch.end());
  RCLCPP_INFO(get_logger(),
    "insertNext: 在 idx=%zu 前插入 %zu 个航点(越障序列),队列总数 %zu",
    current_idx_, batch.size(), targets_.size());
  publishCurrent();
}

std::size_t RouteTargetPublisherNode::currentIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

std::size_t RouteTargetPublisherNode::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

void RouteTargetPublisherNode::setFlightMode(bool active, double flight_z_cm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  flight_mode_ = active;
  flight_z_cm_ = flight_z_cm;
  RCLCPP_INFO(get_logger(),
    "setFlightMode: %s (z 覆盖=%.0fcm) —— 后续航点沿原 xy 在空中飞",
    active ? "ON" : "OFF", flight_z_cm_);
  publishCurrent();  // 立即按新 z 重发当前目标,chassis_mux 据此切换
}

// 飞行模式下把 z 顶成 flight_z_cm,xy/yaw 不变;否则原样
Target RouteTargetPublisherNode::effectiveTarget(const Target & t) const
{
  if (!flight_mode_) {
    return t;
  }
  Target e = t;
  e.z_cm = flight_z_cm_;
  return e;
}

void RouteTargetPublisherNode::publishCurrent(bool verbose)
{
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ < targets_.size()) {
    publishTarget(effectiveTarget(targets_[current_idx_]), current_idx_ == 0, verbose);
  }
}

void RouteTargetPublisherNode::publishTarget(const Target & target, bool init_flag, bool verbose)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = static_cast<float>(target.yaw_deg);
  // 追加前视航点 xy(cm):当前点之后的 lookahead_count_ 个。控制器据此取前方前视点、自动圆角。
  for (std::size_t k = 1; k <= lookahead_count_ &&
       current_idx_ != std::numeric_limits<std::size_t>::max() &&
       current_idx_ + k < targets_.size(); ++k) {
    message.data.push_back(static_cast<float>(targets_[current_idx_ + k].x_cm));
    message.data.push_back(static_cast<float>(targets_[current_idx_ + k].y_cm));
  }
  target_pub_->publish(message);
  if (verbose) {
    RCLCPP_INFO(get_logger(),
      "发布目标: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg%s",
      target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
      init_flag ? " (首个)" : "");
  }
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

bool RouteTargetPublisherNode::getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    z_cm = has_height_ ? current_height_cm_ : 0.0;
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "TF 查询失败 (%s->%s): %s", map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

void RouteTargetPublisherNode::is_st_ready_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data == 1 && !ever_received_st_ready_) {
    ever_received_st_ready_ = true;   // 永久置位
    RCLCPP_INFO(this->get_logger(), "收到 /is_st_ready=1，进入飞机模式");
  }
}


bool RouteTargetPublisherNode::isReached(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dxy = std::hypot(dx, dy);
  const double dz = target.z_cm - z_cm;
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);

  // 空中航点(z>20):z 容忍紧(高度必须到位);地面航点:z 容忍松(忽略地面噪声,
  // 但松到的有限值仍能拦住“从空中降回地面前就误判到达”)。
  const bool airborne = target.z_cm > 20.0;
  const double z_tol = airborne ? air_z_tol_cm_ : ground_z_tol_cm_;
  const bool z_ok = (std::fabs(dz) <= z_tol);
  const bool xy_ok = (dxy <= pos_tol_cm_);
  const bool yaw_ok = (std::fabs(dyaw) <= yaw_tol_deg_);

  if (airborne) {
    // 空中航点也必须等 yaw 到位。follow 路线把每个角点拆成“到点保持来向 +
    // 同 xy/z 原地转向”两个航点；若这里忽略 yaw，第二个原地转航点会被立即跳过，
    // 下一段就会边移动边转头，破坏任务铁律。
    return z_ok && xy_ok && yaw_ok;
  }

  // 地面航点:飞车锁 yaw=0 沿 x 直行,横向(y)修不了、还会漂 30cm ——
  // 若按 xy 一起判,y 一漂就永远到不了,飞车会冲过头(07-16 实测)。所以地面段
  // **只看 x 到位**(沿行进方向),不看 y、不看 yaw;起飞点由 mission 用当前真实 x/y 就地插。
  if (ground_reach_x_only_) {
    const bool x_ok = (std::fabs(dx) <= pos_tol_cm_);
    return z_ok && x_ok;
  }
  // 兼容旧行为:xy + yaw + (松)z 都要满足
  return z_ok && xy_ok && yaw_ok;
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_idx_ == std::numeric_limits<std::size_t>::max() || current_idx_ >= targets_.size()) {
    return;
  }

  // 心跳:每个 tick 重发当前目标(静默,不打日志)。/target_position 虽是锁存单发,
  // 但下游 diff_drive_controller 有 target_timeout_s 安全超时——没有持续心跳它会在 ~2s 后
  // 判目标过期并永久沉默(车停死)。周期重发让它一直追,同时保留"发布器一挂心跳停、
  // 控制器安全停车"的语义。
  publishCurrent(/*verbose=*/false);

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  const Target target = effectiveTarget(targets_[current_idx_]);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "当前目标 %zu: x=%.1f,y=%.1f,z=%.1f,yaw=%.1f",
    current_idx_, target.x_cm, target.y_cm, target.z_cm, target.yaw_deg
  );
  if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
    const Target reached_orig = targets_[current_idx_];  // 原航点(含 land_after 标志与原 xy)
    RCLCPP_INFO(get_logger(),
      "目标 %zu 已完成，准备下一个", current_idx_);
    current_idx_++;

    // 带 land_after 标志且当前在飞:到达后原地垂直下降回地面。
    // 退出 z 覆盖(落点用真实低 z),插一个同 xy 的下降航点;已持锁,直接置 flight_mode_。
    if (reached_orig.land_after && flight_mode_) {
      flight_mode_ = false;
      Target descent{reached_orig.x_cm, reached_orig.y_cm, land_z_cm_, reached_orig.yaw_deg, false};
      targets_.insert(targets_.begin() + static_cast<std::ptrdiff_t>(current_idx_), descent);
      RCLCPP_INFO(get_logger(),
        "到达落点航点(%.1f,%.1f):退出飞行模式,插原地下降点 z=%.0fcm",
        reached_orig.x_cm, reached_orig.y_cm, land_z_cm_);
    }

    if (current_idx_ < targets_.size()) {
      publishCurrent();
    } else {
      current_idx_ = targets_.size();
      RCLCPP_INFO(get_logger(), "所有目标已完成");
    }
  }
}

double RouteTargetPublisherNode::meterToCm(double value_m)
{
  return value_m * 100.0;
}

double RouteTargetPublisherNode::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg) const
{
  const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(normalized);
}


RouteTestNode::RouteTestNode(
  const std::shared_ptr<RouteTargetPublisherNode> & route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("route_test_node", options),
  route_node_(route_node)
{
  std::setlocale(LC_ALL, "");

  // 航点走参数,便于 launch 配置不同测试(跑方形/飞方形)而不必改代码重编。
  // 扁平数组每 4 个一组 [x_cm, y_cm, z_cm, yaw_deg],与 /target_position 布局一致。
  // 默认沿用原演示序列(先前进 2m,升到 100cm 飞方形,再降落),保证 demo1 行为不变。
  const std::vector<double> default_wp{
    200.0, 0.0, 4.0, 0.0,
    200.0, 0.0, 100.0, 0.0,
    200.0, 200.0, 100.0, 0.0,
    0.0, 200.0, 100.0, 0.0,
    0.0, 200.0, 0.0, 0.0};
  // preload_waypoints:=false = 不预装任何航点,起来就静静等 route_topic 下发
  // (terminal 现场规划走这条)。**Tier0/巡航的 launch 必须传它** —— 否则会先按上面
  // 那条演示航点(前进 2m、升到 100cm 飞方形)飞出去,规划的路线还没到就已经起飞了。
  //
  // ⚠ 为什么不是 waypoints:=[]:空列表根本传不进 ROS 2 参数系统 ——
  //   launch 对 [] 推断不出元素类型,直接抛 "Expected 'value' to be one of
  //   [float,int,str,bool,bytes], but got '()'",整个 TimerAction 批全起不来;
  //   就算绕过 launch(ParameterValue/TextSubstitution),YAML 的空序列到了 rcl 也是
  //   PARAMETER_NOT_SET,declare_parameter<vector<double>> 拿到的是类型不符的覆盖值。
  //   2026-07-16 上板实测:现象是 carto/桥/yolo 都正常(位置能回传)、飞车就是不动。
  const bool preload = declare_parameter<bool>("preload_waypoints", true);
  const auto flat = declare_parameter<std::vector<double>>("waypoints", default_wp);

  if (!preload || flat.empty()) {
    RCLCPP_INFO(get_logger(), "不预装航点:等 route_topic 下发路线。");
    return;
  }

  if (flat.size() < 4 || flat.size() % 4 != 0) {
    RCLCPP_FATAL(get_logger(),
      "waypoints 必须是 4 的倍数 [x_cm,y_cm,z_cm,yaw_deg,...],当前 %zu 个数", flat.size());
    throw std::runtime_error("invalid waypoints parameter");
  }

  // 一次性把全部航点压入队列;RouteTargetPublisher 按到达自动推进。
  for (std::size_t i = 0; i + 3 < flat.size(); i += 4) {
    const Target t{flat[i], flat[i + 1], flat[i + 2], flat[i + 3]};
    route_node_->addTarget(t);
    RCLCPP_INFO(get_logger(),
      "添加航点 %zu: x=%.1f y=%.1f z=%.1f yaw=%.1f",
      i / 4, t.x_cm, t.y_cm, t.z_cm, t.yaw_deg);
  }

  RCLCPP_INFO(get_logger(), "Route test node 启动,共加载 %zu 个航点。", flat.size() / 4);
}

}  // namespace activity_control_pkg
