#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <thread>
#include <atomic>
#include <algorithm>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <numeric>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <rclcpp/timer.hpp> // 1. 包含 Timer 头文件

// 包含 TF2 和 geometry_msgs 相关的头文件
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <rclcpp/time.hpp>

class BluetoothNode : public rclcpp::Node
{
public:
    BluetoothNode()
    : Node("bluetooth_node"), serial_df_(-1),
      last_animal_type_(-1), consecutive_count_(0) // 2. 移除 running_ 的初始化
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB2");
        this->declare_parameter<int>("baudrate", 9600);

        std::string port = this->get_parameter("port").as_string();

        bt_data_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("bluetooth_data", 10);

        bt_command_subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "openmv_data", 10, std::bind(&BluetoothNode::sendCallback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        if(openAndConfigureSerialPort(port)) {
            // 3. 移除线程创建，改为创建定时器
            // 定时器每 10 毫秒调用一次 readerCallback 函数
            reader_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&BluetoothNode::readerCallback, this)
            );
            RCLCPP_INFO(this->get_logger(), "Bluetooth node started. Listening on %s.", port.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Node startup failed. Shutting down.");
            if (rclcpp::ok()) {
                rclcpp::shutdown();
            }
        }
    }
    ~BluetoothNode()
    {
        // 4. 移除线程相关的清理代码
        if (serial_df_ >= 0) {
            close(serial_df_);
            RCLCPP_INFO(this->get_logger(), "Bluetooth port closed.");
        }
    }

private:
    bool openAndConfigureSerialPort(const std::string& port) {
        serial_df_ = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (serial_df_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %d from open: %s", errno, strerror(errno));
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Port %s opened.", port.c_str());

        struct termios tty;
        if(tcgetattr(serial_df_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %d from tcgetattr: %s", errno, strerror(errno));
            return false;
        }

        cfmakeraw(&tty);
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(serial_df_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %d from tcsetattr: %s", errno, strerror(errno));
            return false;
        }
        return true;
    }

    // 5. 将 readerLoop 重命名为 readerCallback，并移除 while 循环
    void readerCallback() {
        uint8_t read_buf[256];
        ssize_t n = read(serial_df_, &read_buf, sizeof(read_buf));
        if (n > 0) {
            internal_buffer.insert(internal_buffer.end(), read_buf, read_buf + n);
            processBuffer();
        } else if (n < 0 && errno != EAGAIN) {
            RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s. Shutting down timer.", strerror(errno));
            reader_timer_->cancel(); // 出错时停止定时器
        }
        // 当 n == 0 或 n < 0 且 errno == EAGAIN 时，什么都不做，等待下一次定时器回调
    }

    void processBuffer() {
        const uint8_t HEADER = 0xAA;
        const size_t MIN_FRAME_SIZE = 3;

        while (internal_buffer.size() >= MIN_FRAME_SIZE) {
            size_t frame_start_pos = 0;
            while(frame_start_pos < internal_buffer.size() && internal_buffer[frame_start_pos] != HEADER) {
                frame_start_pos++;
            }

            if (frame_start_pos > 0) {
                internal_buffer.erase(internal_buffer.begin(), internal_buffer.begin() + frame_start_pos);
            }

            if (internal_buffer.empty() || internal_buffer.size() < MIN_FRAME_SIZE) {
                break;
            }

            uint8_t data = internal_buffer[1];
            uint8_t checksum = internal_buffer[2];
            uint8_t calc_checksum = HEADER + data;

            if(checksum == calc_checksum) {
                auto msg = std_msgs::msg::UInt8MultiArray();
                msg.data.push_back(data);
                bt_data_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Received valid data: %d", data);
            } else {
                RCLCPP_WARN(this->get_logger(), "Checksum mismatch. Discarding byte.");
            }
            internal_buffer.erase(internal_buffer.begin(), internal_buffer.begin() + MIN_FRAME_SIZE);
        }
    }

    void sendCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
            // 3. 从消息中提取动物类型
        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty openmv_data message.");
            return;
        }
        int animal_type;
        try {
            animal_type = msg->data.at(0);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Invalid animal type received: %s", e.what());
            return;
        }

        if (animal_type == last_animal_type_) {
            consecutive_count_++;
        } else {
            consecutive_count_ = 1;
            last_animal_type_ = animal_type;
        }
        RCLCPP_INFO(this->get_logger(), "Animal type: %d, Consecutive count: %d/3", animal_type, consecutive_count_);

        if (consecutive_count_ < 3) {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Condition met for animal type %d, calculating pose.", animal_type);
        consecutive_count_ = 0;
        last_animal_type_ = -1;

        geometry_msgs::msg::TransformStamped transform_now;
        double vel_x_m_s, vel_y_m_s; // 明确单位：米/秒
        try {
            transform_now = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            rclcpp::Time past = this->get_clock()->now() - rclcpp::Duration::from_seconds(0.1);
            geometry_msgs::msg::TransformStamped transform_past = tf_buffer_->lookupTransform("map", "base_link", past);

            double dt = (this->get_clock()->now() - past).seconds();
            if (dt < 1e-6) { throw tf2::TransformException("Time difference too small"); }

            vel_x_m_s = (transform_now.transform.translation.x - transform_past.transform.translation.x) / dt;
            vel_y_m_s = (transform_now.transform.translation.y - transform_past.transform.translation.y) / dt;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform/velocity: %s", ex.what());
            return;
        }

        const int num_samples = 10;
        double total_future_x_m = 0.0;
        double total_future_y_m = 0.0;
        for (int i = 1; i <= num_samples; ++i) {
            double t_future = 0.5 * (static_cast<double>(i) / num_samples);
            total_future_x_m += transform_now.transform.translation.x + vel_x_m_s * t_future;
            total_future_y_m += transform_now.transform.translation.y + vel_y_m_s * t_future;
        }

        int32_t center_x_cm = static_cast<int32_t>((total_future_x_m / num_samples) * 100.0);
        int32_t center_y_cm = static_cast<int32_t>((total_future_y_m / num_samples) * 100.0);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Predicted position for animal type %d: X=%d cm, Y=%d cm",
            animal_type, center_x_cm, center_y_cm);

        packAndSendData(static_cast<uint8_t>(animal_type), center_x_cm, center_y_cm);
    }

    void packAndSendData(uint8_t animal_type, int32_t x_cm, int32_t y_cm) {
        std::vector<uint8_t> packet;
        packet.push_back(0xAA);
        packet.push_back(9); // 数据长度: type(1) + x(4) + y(4)

        packet.push_back(animal_type);

        uint8_t temp_bytes[4];
        memcpy(temp_bytes, &x_cm, sizeof(int32_t));
        packet.insert(packet.end(), temp_bytes, temp_bytes + sizeof(int32_t));

        memcpy(temp_bytes, &y_cm, sizeof(int32_t));
        packet.insert(packet.end(), temp_bytes, temp_bytes + sizeof(int32_t));

        uint8_t checksum = std::accumulate(packet.begin(), packet.end(), 0);
        packet.push_back(checksum);

        if (serial_df_ >= 0) {
            ssize_t bytes_written = write(serial_df_, packet.data(), packet.size());
            if (bytes_written != static_cast<ssize_t>(packet.size())) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send all bytes over Bluetooth.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Sent data over Bluetooth: Animal Type=%d, X=%d cm, Y=%d cm", animal_type, x_cm, y_cm);
            }
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr bt_data_publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr bt_command_subscription_;
    rclcpp::TimerBase::SharedPtr reader_timer_; // 6. 添加定时器成员变量
    int serial_df_;
    std::vector<uint8_t> internal_buffer;

    // 状态保持成员变量
    int last_animal_type_;
    int consecutive_count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BluetoothNode>());
    rclcpp::shutdown();
    return 0;
}
