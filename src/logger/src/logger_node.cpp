#include "rclcpp/rclcpp.hpp"

class LoggerNode : public rclcpp::Node {
public:
    LoggerNode() : Node("logger_node") {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { RCLCPP_INFO(this->get_logger(), "Logger node running..."); }
        );
    }
private:
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoggerNode>());
    rclcpp::shutdown();
    return 0;
}
