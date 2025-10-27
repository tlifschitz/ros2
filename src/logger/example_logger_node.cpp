#include "logger/LoggerNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoggerNode>());
  rclcpp::shutdown();
  return 0;
}
