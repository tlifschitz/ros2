#pragma once

#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <memory>
#include <string>
#include <chrono>
#include <cstring>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

struct LogEntry {
    uint64_t timestamp_ns;
    uint32_t message_size;
    // Note: Variable length data follows this struct
};

class FastDataLogger : public rclcpp::Node
{
public:
    FastDataLogger();
    ~FastDataLogger();
    
    void log_data(const void* data, size_t size);
    void log_string(const std::string& message);
    
private:
    void writer_thread();
    void create_new_log_file();
    
    std::string log_directory_;
    std::ofstream log_file_;
    std::string current_log_filename_;
    
    // Performance metrics
    size_t bytes_written_;
    size_t entries_logged_;
    std::atomic<bool> should_stop_;
    std::chrono::steady_clock::time_point start_time_;
    
    // Async writing
    std::thread writer_thread_;
    std::queue<std::vector<uint8_t>> write_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    
    rclcpp::TimerBase::SharedPtr stats_timer_;
};