#pragma once

#include "rclcpp/rclcpp.hpp"

class LoggerNode : public rclcpp::Node
{
public:
  LoggerNode()
  : Node("logger_node")
  {
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() {RCLCPP_INFO(this->get_logger(), "Logger node running...");}
    );
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};
