// diff_drive_controller — 飞车地面差速跟踪控制器
//
// 仿照 car/follower_pkg/diff_drive_controller.cpp(同款 SR5E1E3 差速底盘,不能横移),
// 增加 /ground_enable 使能门:只有地面态(chassis_mux 使能)才驱动轮子,
// 保证与飞控 PID 互斥(车跑时飞控不动,反之亦然)。
//
//   输入  /target_position(Float32MultiArray [x_cm, y_cm, z_cm, yaw_deg],map 系移动靶)
//         /ground_enable(Bool,chassis_mux 按目标 z 仲裁)
//         TF map←laser_link(自身位姿)
//   输出  /cmd_vel(Twist,linear.x=v m/s, angular.z=w rad/s)→ chassis_bridge 转 $VW
//
// 控制律(carrot-chasing):同 car 版,先对准方位再前进。
// 使能门:!ground_enable 时持续发零速(轮子停),不跑控制律。

#include <cmath>
#include <ctime>
#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

struct P2
{
  double x{0.0};
  double y{0.0};
};

// pure-pursuit 前视点:沿折线 robot→chain[0]→chain[1]… 走 Ld 距离取点;
// 路径总长不足 Ld 则返回末点。robot 逼近当前点时前视点自然切进下一段 → 自动圆角。
P2 lookaheadPoint(P2 robot, const std::vector<P2> & chain, double Ld)
{
  double remaining = Ld;
  P2 prev = robot;
  for (const auto & pt : chain) {
    const double dx = pt.x - prev.x;
    const double dy = pt.y - prev.y;
    const double seg = std::hypot(dx, dy);
    if (seg >= remaining) {
      const double t = (seg > 1e-9) ? (remaining / seg) : 0.0;
      return {prev.x + dx * t, prev.y + dy * t};
    }
    remaining -= seg;
    prev = pt;
  }
  return prev;
}
}  // namespace

class DiffDriveController : public rclcpp::Node
{
public:
  DiffDriveController()
  : Node("diff_drive_controller")
  {
    declare_parameter<double>("kp_v", 1.0);             // v = kp_v * 距离误差(m)
    declare_parameter<double>("v_max_mps", 0.4);
    declare_parameter<double>("kp_w", 1.5);             // w = kp_w * 角度误差(rad)
    declare_parameter<double>("w_max_rps", 1.0);
    // 航向环积分项:消掉左右轮恒定速度差导致的直行跑偏(纯 P 会留常驻航向误差)。
    // 只在小方位误差(实际前进)时积分,大误差转向/到点时清零,防 windup。
    declare_parameter<double>("ki_w", 0.0);             // 积分增益,0=退化为纯 P;上板从 0.3 起调
    declare_parameter<double>("iw_limit_rps", 0.3);     // 积分项输出限幅(rad/s),抗饱和
    // 航向环微分阻尼:万向轮拖距=纯滞后系统,纯 P 收尾必超调/摆动。用 yaw 角速度做负反馈
    // (物理即"别转太猛"),不对 e_h 直接微分(近点方位角 atan2 抖会踢飞)。默认 0=关,上板起调。
    declare_parameter<double>("kd_w", 0.0);             // 微分增益:w -= kd_w * yaw_rate
    declare_parameter<double>("yaw_rate_lpf_alpha", 0.5);  // yaw_rate 估计的一阶低通(0..1,1=不滤波)
    // w 斜率限制:禁止 w 每拍瞬跳(spin→直行切换、e_h 反号),减小对滞后系统的激励。0=不限。
    declare_parameter<double>("w_slew_rps2", 0.0);      // |dw/dt| 上限(rad/s^2)
    // align 原地拧的最小转速:破底盘起步死区,让 yaw 能真拧到 yaw_tol 内(0=关)。仅 align 用,不影响直行。
    declare_parameter<double>("w_min_rps", 0.0);
    // pure-pursuit 前视距离:沿前方路径取此距离处的点做转向目标。大→更平滑但抄近路/切内圈更多;
    // 小→更贴线但近点方位角更敏感。需 route 端 lookahead_count>0 追加前视航点才生效。
    declare_parameter<double>("lookahead_dist_cm", 30.0);
    // 线速度最小地板(直行/直通时,v>0 但低于此值则提到此值),防轮子掉进起步死区卡顿(0=关)。
    declare_parameter<double>("v_min_mps", 0.0);
    // 调参用:非空则把每拍内部状态落 CSV(默认空=关);跑完 scp 文件回来分析
    declare_parameter<std::string>("log_csv_path", "");
    declare_parameter<double>("align_gate_deg", 45.0);  // 方位误差超过此值先原地转向
    declare_parameter<double>("pos_tol_cm", 5.0);
    declare_parameter<double>("yaw_tol_deg", 8.0);
    declare_parameter<double>("target_timeout_s", 1.0);
    declare_parameter<double>("stop_burst_s", 1.0);     // 超时后零速发送时长
    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<bool>("ground_enable_default", false);  // 无 mux 时的默认态(安全起见默认禁用)
    // 控制点偏移(车体系,雷达→前驱动轮轴中点)。底盘 $VW 的 v/w 定义在前轴中点上,
    // 把控制点从雷达挪到该点 → 旋转中心与控制点重合,转弯/原地转控制点不平移,走弧线精确。
    // 飞车实测:轴在雷达前方 10.5cm、横向 0(值在 ground_chassis.launch.py 给定)。
    declare_parameter<double>("ctrl_offset_x_cm", 0.0);
    declare_parameter<double>("ctrl_offset_y_cm", 0.0);

    kp_v_ = get_parameter("kp_v").as_double();
    v_max_ = get_parameter("v_max_mps").as_double();
    kp_w_ = get_parameter("kp_w").as_double();
    w_max_ = get_parameter("w_max_rps").as_double();
    ki_w_ = get_parameter("ki_w").as_double();
    iw_limit_ = get_parameter("iw_limit_rps").as_double();
    kd_w_ = get_parameter("kd_w").as_double();
    yaw_rate_alpha_ = clamp(get_parameter("yaw_rate_lpf_alpha").as_double(), 0.0, 1.0);
    w_slew_ = get_parameter("w_slew_rps2").as_double();
    w_min_ = get_parameter("w_min_rps").as_double();
    lookahead_m_ = get_parameter("lookahead_dist_cm").as_double() / 100.0;
    v_min_ = get_parameter("v_min_mps").as_double();
    align_gate_rad_ = get_parameter("align_gate_deg").as_double() * M_PI / 180.0;
    pos_tol_m_ = get_parameter("pos_tol_cm").as_double() / 100.0;
    yaw_tol_rad_ = get_parameter("yaw_tol_deg").as_double() * M_PI / 180.0;
    target_timeout_s_ = get_parameter("target_timeout_s").as_double();
    stop_burst_s_ = get_parameter("stop_burst_s").as_double();
    ground_enabled_ = get_parameter("ground_enable_default").as_bool();
    ctrl_offset_x_m_ = get_parameter("ctrl_offset_x_cm").as_double() / 100.0;
    ctrl_offset_y_m_ = get_parameter("ctrl_offset_y_cm").as_double() / 100.0;
    const double rate_hz = get_parameter("publish_rate_hz").as_double();
    control_dt_ = 1.0 / std::max(rate_hz, 1.0);  // 定频 wall timer,积分步长用标称周期

    std::string csv_path = get_parameter("log_csv_path").as_string();
    if (!csv_path.empty()) {
      // 以 '/' 结尾当目录:自动拼带时间戳的文件名,避免多次跑互相覆盖
      if (csv_path.back() == '/') {
        std::time_t tt = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "ddc_%Y%m%d_%H%M%S.csv", std::localtime(&tt));
        csv_path += buf;
      }
      csv_.open(csv_path, std::ios::out | std::ios::trunc);
      if (csv_.is_open()) {
        csv_ << "t_s,phase,d_cm,e_h_deg,integral,i_term_rps,v_mps,w_rps,"
                "yaw_rate_dps,yaw_deg,self_x,self_y,tgt_x,tgt_y\n";
        log_t0_ = now();
        RCLCPP_INFO(get_logger(), "logging control state -> %s", csv_path.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "cannot open log_csv_path: %s", csv_path.c_str());
      }
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    target_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_position", rclcpp::QoS(10),
      std::bind(&DiffDriveController::targetCallback, this, std::placeholders::_1));

    // 使能门:latched(transient_local),晚启动也能拿到 mux 当前态
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/ground_enable", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&DiffDriveController::enableCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));

    const double period_sec = 1.0 / std::max(rate_hz, 1.0);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec)),
      std::bind(&DiffDriveController::controlTimerCallback, this));

    // 运行时热调:ros2 param set /diff_drive_controller <名> <值> 即时生效,免重编译/重启,调增益用
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&DiffDriveController::onParamUpdate, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "diff_drive_controller up (kp_v=%.2f v_max=%.2f kp_w=%.2f ki_w=%.2f kd_w=%.2f iw_lim=%.2f w_max=%.2f w_slew=%.2f gate=%.0fdeg) ground_enable=%d",
      kp_v_, v_max_, kp_w_, ki_w_, kd_w_, iw_limit_, w_max_, w_slew_, align_gate_rad_ * 180.0 / M_PI, ground_enabled_);
  }

  // 参数热更新:把变动的 declare 过的增益写回缓存成员(节点原本只在构造时读一次)。
  // 角度/长度类参数在此处一并做单位换算,和构造函数里保持一致。
  rcl_interfaces::msg::SetParametersResult onParamUpdate(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      const std::string & n = p.get_name();
      if (n == "kp_v") kp_v_ = p.as_double();
      else if (n == "v_max_mps") v_max_ = p.as_double();
      else if (n == "kp_w") kp_w_ = p.as_double();
      else if (n == "w_max_rps") w_max_ = p.as_double();
      else if (n == "ki_w") ki_w_ = p.as_double();
      else if (n == "iw_limit_rps") iw_limit_ = p.as_double();
      else if (n == "kd_w") kd_w_ = p.as_double();
      else if (n == "yaw_rate_lpf_alpha") yaw_rate_alpha_ = clamp(p.as_double(), 0.0, 1.0);
      else if (n == "w_slew_rps2") w_slew_ = p.as_double();
      else if (n == "w_min_rps") w_min_ = p.as_double();
      else if (n == "lookahead_dist_cm") lookahead_m_ = p.as_double() / 100.0;
      else if (n == "v_min_mps") v_min_ = p.as_double();
      else if (n == "align_gate_deg") align_gate_rad_ = p.as_double() * M_PI / 180.0;
      else if (n == "pos_tol_cm") pos_tol_m_ = p.as_double() / 100.0;
      else if (n == "yaw_tol_deg") yaw_tol_rad_ = p.as_double() * M_PI / 180.0;
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

private:
  void targetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "target_position requires 4 floats [x_cm, y_cm, z_cm, yaw_deg]");
      return;
    }
    target_x_m_ = static_cast<double>(msg->data[0]) / 100.0;
    target_y_m_ = static_cast<double>(msg->data[1]) / 100.0;
    target_yaw_rad_ = static_cast<double>(msg->data[3]) * M_PI / 180.0;
    // 第 4 位起是 pure-pursuit 前视航点(每 2 个一组 xy,cm);没有则退化为纯末点趋近。
    next_pts_.clear();
    for (std::size_t i = 4; i + 1 < msg->data.size(); i += 2) {
      next_pts_.push_back({static_cast<double>(msg->data[i]) / 100.0,
                           static_cast<double>(msg->data[i + 1]) / 100.0});
    }
    last_target_time_ = now();
    has_target_ = true;
  }

  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data != ground_enabled_) {
      RCLCPP_INFO(get_logger(), "ground_enable -> %s", msg->data ? "true(地面态)" : "false(飞控态/停)");
    }
    ground_enabled_ = msg->data;
  }

  bool getCurrentPose(double & x, double & y, double & yaw)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform("map", "laser_link", tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;

      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      // 雷达位姿 → 前驱动轮轴中点(控制点=旋转中心):沿车体系平移偏移量
      x += ctrl_offset_x_m_ * std::cos(yaw) - ctrl_offset_y_m_ * std::sin(yaw);
      y += ctrl_offset_x_m_ * std::sin(yaw) + ctrl_offset_y_m_ * std::cos(yaw);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF map->laser_link unavailable: %s", ex.what());
      return false;
    }
  }

  void publishCmd(double v, double w)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = v;
    msg.angular.z = w;
    cmd_vel_pub_->publish(msg);
  }

  // 停车/失能/丢定位时清控制状态:航向积分、w 斜率基准、yaw_rate 估计一并归零,
  // 避免下次起步时 D 阻尼吃到跨越停顿的旧 yaw 差、或 w 从残留值慢爬。
  void resetControlState()
  {
    iw_integral_ = 0.0;
    last_w_cmd_ = 0.0;
    yaw_rate_filt_ = 0.0;
    have_prev_yaw_ = false;
  }

  void controlTimerCallback()
  {
    // 使能门:非地面态 → 持续发零速(轮子停),不跑控制律,保证与飞控互斥
    if (!ground_enabled_) {
      resetControlState();
      publishCmd(0.0, 0.0);
      return;
    }

    if (!has_target_) {
      return;  // 从未收到目标:保持沉默,不抢 /cmd_vel
    }

    // 目标超时:零速刹停 stop_burst_s,然后转入沉默,等新目标
    const double age_s = (now() - last_target_time_).seconds();
    if (age_s > target_timeout_s_) {
      resetControlState();
      if (age_s <= target_timeout_s_ + stop_burst_s_) {
        publishCmd(0.0, 0.0);
      } else if (!silenced_) {
        silenced_ = true;
        RCLCPP_WARN(get_logger(), "target stale %.1fs -> stop burst done, going silent", age_s);
      }
      return;
    }
    silenced_ = false;

    double x, y, yaw;
    if (!getCurrentPose(x, y, yaw)) {
      resetControlState();
      publishCmd(0.0, 0.0);  // 有目标但无定位:宁可停车
      return;
    }

    // yaw 角速度估计(相邻两拍雷达 yaw 差分 + 一阶低通)→ 航向环 D 阻尼用
    double yaw_rate = 0.0;
    if (have_prev_yaw_) {
      yaw_rate = normalizeAngle(yaw - prev_yaw_) / control_dt_;
      yaw_rate_filt_ = yaw_rate_alpha_ * yaw_rate + (1.0 - yaw_rate_alpha_) * yaw_rate_filt_;
    } else {
      yaw_rate_filt_ = 0.0;  // 首拍无历史:阻尼置零,别用脏差分
    }
    prev_yaw_ = yaw;
    have_prev_yaw_ = true;

    const double d = std::hypot(target_x_m_ - x, target_y_m_ - y);

    double v = 0.0;
    double w = 0.0;
    double log_e = 0.0;      // CSV 误差列(pp/chase=方位误差,align=yaw误差)
    double i_term = 0.0;
    const char * phase = "pp";

    if (!next_pts_.empty()) {
      // ===== pure-pursuit 直通(有后续航点=中间点):取前方前视点,巡航通过,不减速不停 =====
      // 前视点(前方 lookahead_m_ 处)消掉"逼近近点→方位角超敏 + v=kp_v·d 塌陷进死区卡顿"两大病根;
      // 折线 = 当前点 + 后续前视航点 → 前视点会自然切进下一段 → 自动圆角。
      std::vector<P2> chain;
      chain.push_back({target_x_m_, target_y_m_});
      for (const auto & p : next_pts_) {
        chain.push_back(p);
      }
      const P2 L = lookaheadPoint({x, y}, chain, lookahead_m_);
      const double e_h = normalizeAngle(std::atan2(L.y - y, L.x - x) - yaw);
      log_e = e_h;
      iw_integral_ = 0.0;
      w = clamp(kp_w_ * e_h - kd_w_ * yaw_rate_filt_, -w_max_, w_max_);
      v = clamp(v_max_ * std::cos(e_h), 0.0, v_max_);          // 巡航,拐急(e_h 大)才降速
      if (v > 1e-3 && v_min_ > 1e-9 && v < v_min_) v = v_min_;  // 防轮子掉进起步死区卡顿
    } else if (d > pos_tol_m_) {
      // ===== 末航点趋近:沿用 carrot(align_gate 门 + 直行积分 + 按 d 降速到停) =====
      phase = "chase";
      const double e_h = normalizeAngle(std::atan2(target_y_m_ - y, target_x_m_ - x) - yaw);
      log_e = e_h;
      if (ki_w_ > 1e-9 && std::fabs(e_h) <= align_gate_rad_) {
        iw_integral_ += e_h * control_dt_;
        iw_integral_ = clamp(iw_integral_, -iw_limit_ / ki_w_, iw_limit_ / ki_w_);
        i_term = ki_w_ * iw_integral_;
        v = clamp(kp_v_ * d, 0.0, v_max_) * std::cos(e_h);
      } else {
        iw_integral_ = 0.0;
        if (std::fabs(e_h) <= align_gate_rad_) {
          v = clamp(kp_v_ * d, 0.0, v_max_) * std::cos(e_h);
        }
      }
      w = clamp(kp_w_ * e_h + i_term - kd_w_ * yaw_rate_filt_, -w_max_, w_max_);
      if (v > 1e-3 && v_min_ > 1e-9 && v < v_min_) v = v_min_;
    } else {
      // ===== 末航点到位:原地对准目标 yaw(带 w_min 破死区,kd 先减后兜底见注) =====
      phase = "align";
      iw_integral_ = 0.0;
      const double e_yaw = normalizeAngle(target_yaw_rad_ - yaw);
      log_e = e_yaw;
      if (std::fabs(e_yaw) > yaw_tol_rad_) {
        // w_min 只兜底给比例项,再减阻尼:转太快(stick-slip 窜)时 kd 仍能把 w 压到 w_min 以下真刹车;
        // 只有快停下 yaw_rate≈0、确实卡死区时 w 才落在 w_min 推过去。放最后当硬地板会顶掉 kd 刹车量。
        double w_cmd = kp_w_ * e_yaw;
        if (w_min_ > 1e-9 && std::fabs(w_cmd) < w_min_) {
          w_cmd = (e_yaw >= 0.0) ? w_min_ : -w_min_;
        }
        w_cmd -= kd_w_ * yaw_rate_filt_;
        w = clamp(w_cmd, -w_max_, w_max_);
      }
    }

    // w 斜率限制:朝目标 w 每拍最多爬 w_slew_*dt,压制猛打方向对滞后系统的激励(0=不限)
    if (w_slew_ > 1e-9) {
      const double dw_max = w_slew_ * control_dt_;
      w = clamp(w, last_w_cmd_ - dw_max, last_w_cmd_ + dw_max);
    }
    last_w_cmd_ = w;

    logRow(phase, d, log_e, i_term, v, w, yaw, x, y);
    publishCmd(v, w);
  }

  // 每拍落一行内部状态(仅 log_csv_path 非空时);角度列转 deg、距离转 cm 便于直接看
  void logRow(const char * phase, double d, double e_rad, double i_term,
              double v, double w, double yaw, double self_x, double self_y)
  {
    if (!csv_.is_open()) {
      return;
    }
    const double t = (now() - log_t0_).seconds();
    csv_ << t << ',' << phase << ',' << d * 100.0 << ','
         << e_rad * 180.0 / M_PI << ',' << iw_integral_ << ',' << i_term << ','
         << v << ',' << w << ',' << yaw_rate_filt_ * 180.0 / M_PI << ','
         << yaw * 180.0 / M_PI << ','
         << self_x << ',' << self_y << ',' << target_x_m_ << ',' << target_y_m_ << '\n';
    csv_.flush();
  }

  // 参数
  double kp_v_, v_max_, kp_w_, w_max_;
  double ki_w_{0.0}, iw_limit_{0.3}, control_dt_{0.05};
  double kd_w_{0.0}, yaw_rate_alpha_{0.5}, w_slew_{0.0}, w_min_{0.0};
  double lookahead_m_{0.3}, v_min_{0.0};
  std::vector<P2> next_pts_;   // pure-pursuit 前视航点(map, m),route 追加而来
  double align_gate_rad_, pos_tol_m_, yaw_tol_rad_;
  double target_timeout_s_, stop_burst_s_;
  double ctrl_offset_x_m_{0.0}, ctrl_offset_y_m_{0.0};

  // 航向环积分累积量(∫e_h dt)
  double iw_integral_{0.0};

  // 航向环 D 阻尼 + w 斜率限制的运行状态
  double prev_yaw_{0.0};
  bool have_prev_yaw_{false};
  double yaw_rate_filt_{0.0};
  double last_w_cmd_{0.0};

  // 调参日志
  std::ofstream csv_;
  rclcpp::Time log_t0_;

  // 状态
  double target_x_m_{0.0}, target_y_m_{0.0}, target_yaw_rad_{0.0};
  bool has_target_{false};
  bool silenced_{false};
  bool ground_enabled_{false};
  rclcpp::Time last_target_time_;

  // ROS 接口
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveController>());
  rclcpp::shutdown();
  return 0;
}
