#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <algorithm>
#include <sstream> // 关键：包含 sstream 用于字符串解析

// 包含 Linux 底层串口所需的头文件
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class OpenMVBridge : public rclcpp::Node
{
public:
    OpenMVBridge() 
    : Node("openmv_bridge"), serial_fd_(-1), running_(false)
    {
        declare_parameter<std::string>("port", "/dev/ttyACM0");
        declare_parameter<int>("baudrate", 9600);

        std::string port = get_parameter("port").as_string();
        
        pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("openmv_data", 10);

        if (!openAndConfigureSerialPort(port)) {
            rclcpp::shutdown();
            return;
        }

        running_.store(true);
        reader_thread_ = std::thread(&OpenMVBridge::readerLoop, this);

        RCLCPP_INFO(this->get_logger(), "OpenMV Bridge Node Started using low-level POSIX driver.");
    }

    ~OpenMVBridge()
    {
        running_.store(false);
        if (reader_thread_.joinable()) {
            reader_thread_.join();
        }
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    bool openAndConfigureSerialPort(const std::string& port) {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened.", port.c_str());

        struct termios tty;
        if(tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
            return false;
        }

        cfmakeraw(&tty);
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
            return false;
        }
        return true;
    }

    void readerLoop() {
        char read_buf[256];
        while(running_.load()) {
            ssize_t n = read(serial_fd_, &read_buf, sizeof(read_buf));
            if (n > 0) {
                internal_buffer_.insert(internal_buffer_.end(), read_buf, read_buf + n);
                processBuffer();
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    // ==================== 关键修改：解析 "0xaa 0x5 0xaf" 格式 ====================
    void processBuffer() {
        while (true) {
            auto it = std::find(internal_buffer_.begin(), internal_buffer_.end(), '\n');
            if (it == internal_buffer_.end()) {
                break; // 没有完整的行了
            }

            std::string line(internal_buffer_.begin(), it);
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            // 使用 stringstream 来分割由空格隔开的字符串
            std::stringstream ss(line);
            std::string header_str, payload_str, checksum_str;

            // 尝试读取三个部分
            if (ss >> header_str >> payload_str >> checksum_str) {
                try {
                    // 将字符串（例如 "0x5"）转换为整数。
                    // std::stoi 的第三个参数 0 表示自动检测进制（看到 "0x" 就按16进制处理）
                    int animal_number = std::stoi(payload_str, nullptr, 0);

                    // (可选但推荐) 进行校验和验证，确保数据完整性
                    int header = std::stoi(header_str, nullptr, 0);
                    int received_checksum = std::stoi(checksum_str, nullptr, 0);
                    int calculated_checksum = (header + animal_number) & 0xFF;

                    if (received_checksum == calculated_checksum) {
                        // 校验成功，发布消息
                        auto msg = std_msgs::msg::UInt8MultiArray();
                        msg.data.push_back(static_cast<uint8_t>(animal_number));
                        pub_->publish(msg);
                        RCLCPP_INFO(this->get_logger(), "Parsed '%s'. Checksum OK. Published data: %d", line.c_str(), animal_number);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Checksum error on line '%s'. Received: %d, Calculated: %d", line.c_str(), received_checksum, calculated_checksum);
                    }

                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Error parsing numbers in line '%s': %s", line.c_str(), e.what());
                }
            }
            // 从缓冲区移除已处理的行
            internal_buffer_.erase(internal_buffer_.begin(), it + 1);
        }
    }
    // =============================================================================

    int serial_fd_;
    std::thread reader_thread_;
    std::atomic<bool> running_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_;
    std::vector<uint8_t> internal_buffer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpenMVBridge>());
    rclcpp::shutdown();
    return 0;
}