#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <cstring>
#include <limits>
#include <cmath>
#include <chrono>
#include <functional>
#include <mutex>
#include <atomic>

namespace laser_array {

// 协议常量
namespace protocol {
    constexpr std::uint8_t HEADER[4] = {0x57, 0x01, 0xFF, 0x00};
    constexpr std::size_t HEADER_SIZE = 4;
    constexpr std::size_t TIMESTAMP_SIZE = 4;
    constexpr std::size_t COUNT_SIZE = 1;
    constexpr std::size_t DATA_UNIT_SIZE = 6;
    constexpr std::size_t FOOTER_SIZE = 6;
    constexpr std::size_t CHECKSUM_SIZE = 1;
    constexpr std::uint8_t DATA_COUNT = 16;  // 固定16个数据
    constexpr std::size_t PACKET_SIZE = 112; // 固定帧长度
    constexpr std::size_t MAX_BUFFER_SIZE = 4096;
}

// 主节点类
class LaserArrayNode : public rclcpp::Node {
public:
    explicit LaserArrayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("laser_array_node", options)
        , running_(false)
        , shutdown_complete_(false)
        , buffer_index_(0)
    {
        // 初始化参数
        declare_parameter<std::string>("serial_port", "/dev/ttyS3");
        declare_parameter<int>("baud_rate", 921600);
        declare_parameter<std::string>("frame_id", "laser_frame");
        
        get_parameter("serial_port", serial_port_name_);
        get_parameter("baud_rate", baud_rate_);
        get_parameter("frame_id", frame_id_);
        
        // 初始化发布者
        range_pub_ = create_publisher<std_msgs::msg::Float32>("average_range", 10);
        
        // 初始化缓冲区
        read_buffer_.resize(1024);
        parse_buffer_.resize(protocol::MAX_BUFFER_SIZE);
        
        // 初始化串口
        initializeSerial();
        
        RCLCPP_INFO(this->get_logger(), "LaserArray node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Port: %s, Baud: %d", serial_port_name_.c_str(), baud_rate_);
        RCLCPP_INFO(this->get_logger(), "Fixed packet size: %zu bytes, Data count: %d", 
                   protocol::PACKET_SIZE, protocol::DATA_COUNT);
    }

    ~LaserArrayNode() {
        // 先停止运行标志
        running_ = false;
        
        // 关闭串口
        if (serial_port_ptr_ && serial_port_ptr_->is_open()) {
            try {
                serial_port_ptr_->close();
            } catch (...) {
                // 忽略关闭时的错误
            }
        }
        
        // 等待线程结束
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
        
        // 标记shutdown完成
        shutdown_complete_ = true;
        
        // 使用printf避免ROS logger问题
        printf("LaserArray node shutdown complete\n");
    }

private:
    // 参数
    std::string serial_port_name_;
    int baud_rate_;
    std::string frame_id_;
    
    // 串口组件
    std::shared_ptr<boost::asio::serial_port> serial_port_ptr_;
    boost::asio::io_service io_service_;
    std::thread io_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> shutdown_complete_;
    
    // ROS组件
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr range_pub_;
    
    // 数据处理
    std::vector<std::uint8_t> read_buffer_;
    std::vector<std::uint8_t> parse_buffer_;
    std::size_t buffer_index_;
    std::mutex buffer_mutex_;
    
    void initializeSerial() {
        try {
            serial_port_ptr_ = std::make_shared<boost::asio::serial_port>(io_service_);
            serial_port_ptr_->open(serial_port_name_);
            serial_port_ptr_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
            serial_port_ptr_->set_option(boost::asio::serial_port_base::character_size(8));
            serial_port_ptr_->set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            serial_port_ptr_->set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            serial_port_ptr_->set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));
                
            running_ = true;
            io_thread_ = std::thread(&LaserArrayNode::readSerialThread, this);
            
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            throw;
        }
    }
    
    void readSerialThread() {
        while (running_ && !shutdown_complete_) {
            try {
                if (!serial_port_ptr_->is_open()) {
                    if (running_) {
                        printf("Serial port not open\n");
                    }
                    break;
                }
                
                size_t bytes_read = serial_port_ptr_->read_some(
                    boost::asio::buffer(read_buffer_)
                );
                
                if (bytes_read > 0 && running_ && !shutdown_complete_) {
                    // 线程安全地处理数据
                    {
                        std::lock_guard<std::mutex> lock(buffer_mutex_);
                        appendToBuffer(read_buffer_.data(), bytes_read);
                        processBuffer();
                    }
                }
                
                // 短暂休眠避免占用过多CPU
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                
            } catch (const boost::system::system_error& e) {
                if (running_ && !shutdown_complete_) {
                    printf("Serial read error: %s\n", e.what());
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        printf("Serial read thread exiting\n");
    }
    
    void appendToBuffer(const std::uint8_t* data, std::size_t length) {
        if (buffer_index_ + length > parse_buffer_.size()) {
            printf("Buffer overflow, resetting\n");
            buffer_index_ = 0;
        }
        
        std::memcpy(parse_buffer_.data() + buffer_index_, data, length);
        buffer_index_ += length;
    }
    
    void processBuffer() {
        if (shutdown_complete_) return;
        
        // 持续处理完整的112字节帧
        while (buffer_index_ >= protocol::PACKET_SIZE && running_ && !shutdown_complete_) {
            if (findAndProcessPacket()) {
                //printf("Packet processed successfully\n");
            } else {
                // 未找到有效帧，移动一个字节继续查找
                shiftBuffer(1);
            }
        }
    }
    
    bool findAndProcessPacket() {
        if (shutdown_complete_) return false;
        
        // 在当前缓冲区中查找112字节的完整帧
        for (std::size_t i = 0; i + protocol::PACKET_SIZE <= buffer_index_; ++i) {
            // 检查是否是有效的帧头
            if (std::memcmp(parse_buffer_.data() + i, protocol::HEADER, protocol::HEADER_SIZE) == 0) {
                // 检查数据计数是否为16
                std::uint8_t data_count = parse_buffer_[i + protocol::HEADER_SIZE + protocol::TIMESTAMP_SIZE];
                if (data_count == protocol::DATA_COUNT) {
                    // 验证校验和
                    if (validateChecksum(i)) {
                        // 处理这个完整的帧
                        processPacket(i);
                        // 移除已处理的数据
                        shiftBuffer(i + protocol::PACKET_SIZE);
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    bool validateChecksum(std::size_t start_offset) {
        std::uint8_t checksum = 0;
        for (std::size_t i = 0; i < protocol::PACKET_SIZE - 1; ++i) {
            checksum += parse_buffer_[start_offset + i];
        }
        return checksum == parse_buffer_[start_offset + protocol::PACKET_SIZE - 1];
    }
    
    void processPacket(std::size_t start_offset) {
        if (shutdown_complete_) return;
        
        const std::uint8_t* packet_data = parse_buffer_.data() + start_offset;
        
        // 解析时间戳（小端序）
        std::uint32_t timestamp;
        std::memcpy(&timestamp, packet_data + protocol::HEADER_SIZE, sizeof(timestamp));
        
        // 解析16个距离数据
        std::vector<float> valid_distances;
        float sum = 0.0f;
        int valid_count = 0;
        
        //printf("=== Frame Analysis ===\n");
        //printf("Timestamp: %u\n", timestamp);
        
        for (int i = 0; i < protocol::DATA_COUNT; ++i) {
            std::size_t offset = protocol::HEADER_SIZE + protocol::TIMESTAMP_SIZE + 
                                protocol::COUNT_SIZE + i * protocol::DATA_UNIT_SIZE;
            
            bool is_valid = (packet_data[offset + 3] == 0);
            if (is_valid) {
                float distance = parseDistance(packet_data + offset);
                valid_distances.push_back(distance);
                sum += distance;
                valid_count++;
                
                //printf("Sensor[%02d]: %.6f m\n", i, distance);
            } else {
                //printf("Sensor[%02d]: INVALID\n", i);
            }
        }
        
        // 发布平均值
        if (valid_count > 0 && !shutdown_complete_) {
            float average = sum / valid_count;
            publishRange(average);
            //printf("Valid sensors: %d/%d\n", valid_count, protocol::DATA_COUNT);
            //printf("Average distance: %.6f m\n", average);
        } else {
            if (!shutdown_complete_) {
                publishRange(std::numeric_limits<float>::quiet_NaN());
                printf("No valid distance measurements in frame\n");
            }
        }
        //printf("===================\n");
    }
    
    float parseDistance(const std::uint8_t* bytes) {
        // 小端序解析24位有符号整数
        std::int32_t raw = bytes[0] | (bytes[1] << 8) | (bytes[2] << 16);
        
        // 符号扩展
        if (raw & 0x800000) raw |= 0xFF000000;
        
        return static_cast<float>(raw) / 1000000.0f;
    }
    
    void shiftBuffer(std::size_t offset) {
        if (offset >= buffer_index_) {
            buffer_index_ = 0;
        } else {
            std::memmove(parse_buffer_.data(), 
                        parse_buffer_.data() + offset, 
                        buffer_index_ - offset);
            buffer_index_ -= offset;
        }
    }
    
    void publishRange(float range) {
        if (shutdown_complete_ || !range_pub_) return;
        
        try {
            auto msg = std::make_unique<std_msgs::msg::Float32>();
            msg->data = range;
            range_pub_->publish(std::move(msg));
        } catch (...) {
            // 忽略发布时的错误
        }
    }
};

} // namespace laser_array

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<laser_array::LaserArrayNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        printf("Exception: %s\n", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}