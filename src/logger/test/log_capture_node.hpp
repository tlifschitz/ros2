#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <memory>
#include <vector>
#include <string>

class LogCaptureNode : public rclcpp::Node
{
public:
  LogCaptureNode()
  : Node("log_capture_node")
  {
    // Subscribe to the /rosout topic where all logs are published
    log_subscription_ = this->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 10,
      [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
        captured_messages_.push_back(msg->msg);
      });
  }

  std::vector<std::string> get_captured_messages() const
  {
    return captured_messages_;
  }

  void clear_messages()
  {
    captured_messages_.clear();
  }

private:
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_subscription_;
  std::vector<std::string> captured_messages_;
};
