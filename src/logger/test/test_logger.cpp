#include <memory>
#include <vector>
#include <string>
#include "logger/LoggerNode.hpp"
#include "log_capture_node.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

class SingleThreadedExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

class LoggerTest : public SingleThreadedExecutorTest {};

TEST_F(LoggerTest, NodeCreatesSuccessfully) {
  auto node = std::make_shared<LoggerNode>();

  EXPECT_NE(node, nullptr);
  EXPECT_EQ(node->get_name(), std::string("logger_node"));
}

TEST_F(LoggerTest, NodeSpinsWithoutErrors) {
  auto logger_node = std::make_shared<LoggerNode>();

  executor_->add_node(logger_node);

  auto start_time = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(200)) {
    executor_->spin_some(std::chrono::milliseconds(50));
  }

  EXPECT_TRUE(true);
}

TEST_F(LoggerTest, EmitsCorrectLogMessage) {
  auto logger_node = std::make_shared<LoggerNode>();
  auto log_capture_node = std::make_shared<LogCaptureNode>();

  executor_->add_node(logger_node);
  executor_->add_node(log_capture_node);

  log_capture_node->clear_messages();

  // Spin for enough time to capture at least one log message
  auto start_time = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
    executor_->spin_some(std::chrono::milliseconds(50));
  }

  // Check if we've captured the expected message
  auto messages = log_capture_node->get_captured_messages();

  for (const auto & msg : messages) {
    if (msg.find("Logger node running...") != std::string::npos) {
      SUCCEED();    // Test passed
      return;
    }
  }

  // If we get here, we didn't find the expected message
  RCLCPP_INFO(rclcpp::get_logger("test"), "Captured %zu messages", messages.size());
  for (const auto & msg : messages) {
    RCLCPP_INFO(rclcpp::get_logger("test"), "Message: %s", msg.c_str());
  }

  FAIL() << "Expected log message 'Logger node running...' was not found";
}
