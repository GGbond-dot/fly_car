#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace serial_comm {

/**
 * @brief 串口通讯异常类
 */
class SerialException : public std::exception {
public:
    explicit SerialException(const std::string& message) : message_(message) {}
    const char* what() const noexcept override { return message_.c_str(); }

private:
    std::string message_;
};

/**
 * @brief 协议帧结构定义
 */
struct ProtocolFrame {
    static const uint8_t FRAME_HEADER = 0xAA;  // 帧头
    static const uint8_t ADDRESS = 0xFF;       // 地址
    static const size_t MIN_FRAME_SIZE = 6;    // 最小帧长度（帧头+地址+ID+长度+校验2字节）
    static const size_t MAX_DATA_SIZE = 255;   // 最大数据长度
    
    uint8_t id;                                // 数据类型ID
    uint8_t data_length;                       // 数据长度
    std::vector<uint8_t> data;                 // 实际数据
    uint8_t sum_check;                         // 和校验
    uint8_t add_check;                         // 附加校验
    
    ProtocolFrame() : id(0), data_length(0), sum_check(0), add_check(0) {}
};

/**
 * @brief 协议数据处理回调函数类型定义
 */
using ProtocolDataHandler = std::function<void(uint8_t id, const std::vector<uint8_t>& data)>;

/**
 * @brief 串口参数配置结构体
 */
struct SerialConfig {
    std::string port_name;          // 串口名称，如 "/dev/ttyUSB0"
    unsigned int baud_rate;         // 波特率
    unsigned int character_size;    // 数据位，默认8位
    boost::asio::serial_port::parity::type parity;         // 校验位，默认无校验
    boost::asio::serial_port::stop_bits::type stop_bits;   // 停止位，默认1位
    boost::asio::serial_port::flow_control::type flow_control; // 流控制，默认无流控

    SerialConfig() :
        port_name(""),
        baud_rate(9600),
        character_size(8),
        parity(boost::asio::serial_port::parity::none),
        stop_bits(boost::asio::serial_port::stop_bits::one),
        flow_control(boost::asio::serial_port::flow_control::none) {}
};

/**
 * @brief 串口通讯类
 * 基于boost::asio实现的高性能串口通讯库
 * 支持同步和异步读写操作
 */
class SerialComm {
public:
    // 数据接收回调函数类型定义
    using DataReceivedCallback = std::function<void(const std::vector<uint8_t>&)>;
    using ErrorCallback = std::function<void(const std::string&)>;

    /**
     * @brief 构造函数
     */
    SerialComm();

    /**
     * @brief 析构函数
     */
    ~SerialComm();

    /**
     * @brief 万能串口初始化函数
     * @param port_name 串口名称，如 "/dev/ttyUSB0", "/dev/ttyACM0" 等
     * @param baud_rate 波特率，如 9600, 115200 等
     * @return true 初始化成功，false 初始化失败
     */
    bool initialize(const std::string& port_name, unsigned int baud_rate);

    /**
     * @brief 高级初始化函数
     * @param config 串口配置参数
     * @return true 初始化成功，false 初始化失败
     */
    bool initialize(const SerialConfig& config);

    /**
     * @brief 检查串口是否已打开
     * @return true 已打开，false 未打开
     */
    bool is_open() const;

    /**
     * @brief 关闭串口
     */
    void close();

    /**
     * @brief 同步写入数据
     * @param data 要写入的数据
     * @return 实际写入的字节数，-1表示失败
     */
    int write(const std::vector<uint8_t>& data);

    /**
     * @brief 同步写入字符串数据
     * @param data 要写入的字符串
     * @return 实际写入的字节数，-1表示失败
     */
    int write(const std::string& data);

    /**
     * @brief 同步读取数据
     * @param buffer 读取数据的缓冲区
     * @param max_size 最大读取字节数
     * @param timeout_ms 超时时间（毫秒），0表示无超时
     * @return 实际读取的字节数，-1表示失败
     */
    int read(std::vector<uint8_t>& buffer, size_t max_size, unsigned int timeout_ms = 0);

    /**
     * @brief 同步读取一行数据（以\n结尾）
     * @param line 读取的行数据
     * @param timeout_ms 超时时间（毫秒），0表示无超时
     * @return true 读取成功，false 读取失败
     */
    bool read_line(std::string& line, unsigned int timeout_ms = 0);

    /**
     * @brief 异步写入数据
     * @param data 要写入的数据
     * @param callback 写入完成后的回调函数
     */
    void async_write(const std::vector<uint8_t>& data, 
                     std::function<void(const boost::system::error_code&, size_t)> callback = nullptr);

    /**
     * @brief 开始异步读取数据
     * @param callback 数据接收回调函数
     * @param error_callback 错误回调函数
     */
    void start_async_read(DataReceivedCallback callback, ErrorCallback error_callback = nullptr);

    /**
     * @brief 停止异步读取
     */
    void stop_async_read();

    /**
     * @brief 清空接收缓冲区
     */
    void flush_input();

    /**
     * @brief 清空发送缓冲区
     */
    void flush_output();

    /**
     * @brief 获取可用的串口列表
     * @return 串口名称列表
     */
    static std::vector<std::string> get_available_ports();

    /**
     * @brief 设置读取超时时间
     * @param timeout_ms 超时时间（毫秒）
     */
    void set_read_timeout(unsigned int timeout_ms);

    /**
     * @brief 获取最后的错误信息
     * @return 错误信息字符串
     */
    std::string get_last_error() const;

    /**
     * @brief 按协议发送数据（同步发送）
     * @param id 数据类型ID（帧ID）
     * @param data_length 数据长度
     * @param data 要发送的数据内容
     * @return true 发送成功，false 发送失败
     */
    bool send_protocol_data(uint8_t id, uint8_t data_length, const std::vector<uint8_t>& data);

    /**
     * @brief 开始协议数据接收（异步）
     * @param handler 协议数据处理回调函数
     * @param error_callback 错误回调函数
     */
    void start_protocol_receive(ProtocolDataHandler handler, ErrorCallback error_callback = nullptr);

    /**
     * @brief 停止协议数据接收
     */
    void stop_protocol_receive();

private:
    // 内部方法
    void async_read_handler(const boost::system::error_code& error, size_t bytes_transferred);
    void start_io_service();
    void stop_io_service();
    bool configure_port(const SerialConfig& config);

    // 协议相关的内部方法
    std::vector<uint8_t> build_protocol_frame(uint8_t id, const std::vector<uint8_t>& data);
    void calculate_checksum(const std::vector<uint8_t>& frame_data, uint8_t& sum_check, uint8_t& add_check);
    bool parse_protocol_frame(std::vector<uint8_t>& buffer);
    void protocol_data_received(const std::vector<uint8_t>& data);

    // 成员变量
    std::unique_ptr<boost::asio::io_service> io_service_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::unique_ptr<std::thread> io_thread_;
    
    std::vector<uint8_t> read_buffer_;
    static const size_t READ_BUFFER_SIZE = 1024;
    
    DataReceivedCallback data_callback_;
    ErrorCallback error_callback_;
    
    std::atomic<bool> is_async_reading_;
    std::atomic<bool> is_running_;
    
    unsigned int read_timeout_ms_;
    mutable std::mutex error_mutex_;
    std::string last_error_;
    
    SerialConfig current_config_;
    
    // 协议相关的成员变量
    ProtocolDataHandler protocol_handler_;
    std::vector<uint8_t> protocol_buffer_;     // 协议解析缓冲区
    std::mutex protocol_mutex_;                // 协议处理互斥锁
};

} // namespace serial_comm

#endif // SERIAL_COMM_H
