#include <gtest/gtest.h>
#include "logger/FastDataLogger.hpp"
#include <chrono>
#include <random>
#include <filesystem>
#include <cstdlib>

class LoggerPerformanceTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        logger_ = std::make_shared<FastDataLogger>();
        
        // Detect if running in Docker or container environment
        is_containerized_ = isRunningInContainer();
        if (is_containerized_) {
            std::cout << "Detected containerized environment - adjusting performance expectations" << std::endl;
        }
    }
    
    void TearDown() override
    {
        logger_.reset();
        rclcpp::shutdown();
        
        // Clean up test files
        if (std::filesystem::exists("./logs")) {
            std::filesystem::remove_all("./logs");
        }
    }
    
    bool isRunningInContainer() const
    {
        // Check for common container indicators
        return std::filesystem::exists("/.dockerenv") ||
               std::getenv("DOCKER_CONTAINER") != nullptr ||
               std::getenv("CONTAINER") != nullptr;
    }
    
    std::shared_ptr<FastDataLogger> logger_;
    bool is_containerized_ = false;
};

TEST_F(LoggerPerformanceTest, HighThroughputTest)
{
    const int num_messages = 10000;
    const size_t message_size = 1024;  // 1KB messages
    
    std::vector<uint8_t> test_data(message_size);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    
    // Fill with random data
    for (auto& byte : test_data) {
        byte = static_cast<uint8_t>(dis(gen));
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_messages; ++i) {
        logger_->log_data(test_data.data(), test_data.size());
    }
    
    // Allow time for async writing to complete
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    double throughput_mb = (num_messages * message_size) / 1024.0 / 1024.0 / (duration.count() / 1000.0);
    double messages_per_sec = num_messages / (duration.count() / 1000.0);
    
    std::cout << "Performance Results:" << std::endl;
    std::cout << "Messages: " << num_messages << std::endl;
    std::cout << "Message size: " << message_size << " bytes" << std::endl;
    std::cout << "Duration: " << duration.count() << " ms" << std::endl;
    std::cout << "Throughput: " << throughput_mb << " MB/s" << std::endl;
    std::cout << "Messages/sec: " << messages_per_sec << std::endl;
    
    // Performance assertions - adjusted for container environments
    double min_throughput = is_containerized_ ? 2.0 : 10.0;  // Lower expectations in containers
    double min_msg_rate = is_containerized_ ? 500.0 : 1000.0;
    
    std::cout << "Environment: " << (is_containerized_ ? "Container" : "Native") << std::endl;
    std::cout << "Expected min throughput: " << min_throughput << " MB/s" << std::endl;
    std::cout << "Expected min message rate: " << min_msg_rate << " messages/sec" << std::endl;
    
    EXPECT_GT(throughput_mb, min_throughput);
    EXPECT_GT(messages_per_sec, min_msg_rate);
}

TEST_F(LoggerPerformanceTest, ConcurrentLoggingTest)
{
    const int num_threads = 4;
    const int messages_per_thread = 1000;
    const std::string test_message = "Concurrent test message with some data";
    
    auto start = std::chrono::high_resolution_clock::now();
    
    std::vector<std::thread> threads;
    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([this, messages_per_thread, &test_message, t]() {
            for (int i = 0; i < messages_per_thread; ++i) {
                std::string msg = test_message + " from thread " + std::to_string(t) + " msg " + std::to_string(i);
                logger_->log_string(msg);
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    // Allow time for async writing
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    int total_messages = num_threads * messages_per_thread;
    double messages_per_sec = total_messages / (duration.count() / 1000.0);
    
    std::cout << "Concurrent test results:" << std::endl;
    std::cout << "Threads: " << num_threads << std::endl;
    std::cout << "Total messages: " << total_messages << std::endl;
    std::cout << "Duration: " << duration.count() << " ms" << std::endl;
    std::cout << "Messages/sec: " << messages_per_sec << std::endl;
    
    // Adjusted expectations for container environments
    double min_concurrent_rate = is_containerized_ ? 250.0 : 500.0;
    std::cout << "Expected min concurrent rate: " << min_concurrent_rate << " messages/sec" << std::endl;
    
    EXPECT_GT(messages_per_sec, min_concurrent_rate);
}

TEST_F(LoggerPerformanceTest, DataIntegrityTest)
{
    // Test that focuses on data integrity rather than pure performance
    const int num_messages = 1000;
    std::vector<std::string> test_messages;
    
    // Generate test messages
    for (int i = 0; i < num_messages; ++i) {
        test_messages.push_back("Test message " + std::to_string(i) + " with unique content");
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Log all messages
    for (const auto& msg : test_messages) {
        logger_->log_string(msg);
    }
    
    // Allow time for all writes to complete
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "Data Integrity Test Results:" << std::endl;
    std::cout << "Messages logged: " << num_messages << std::endl;
    std::cout << "Duration: " << duration.count() << " ms" << std::endl;
    
    // Check that log file was created and contains data
    bool found_log_file = false;
    size_t total_file_size = 0;
    
    if (std::filesystem::exists("./logs")) {
        for (const auto& entry : std::filesystem::directory_iterator("./logs")) {
            if (entry.path().extension() == ".bin") {
                found_log_file = true;
                total_file_size = std::filesystem::file_size(entry.path());
                std::cout << "Log file: " << entry.path() << " (" << total_file_size << " bytes)" << std::endl;
                break;
            }
        }
    }
    
    EXPECT_TRUE(found_log_file) << "Log file should be created";
    EXPECT_GT(total_file_size, 0) << "Log file should contain data";
    
    // Verify reasonable file size (each entry has header + message)
    size_t expected_min_size = num_messages * (sizeof(LogEntry) + 10); // At least 10 bytes per message
    EXPECT_GT(total_file_size, expected_min_size) << "Log file size should be reasonable";
}