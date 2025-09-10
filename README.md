# ROS Project
# Skeleton
## Container Setup

1. Install Docker
2. Pull ROS2 Humble image (Ubuntu 22.04):

```bash
docker pull osrf/ros:humble-desktop
```

3. Start a dev container
   
```bash
docker run -it --rm \
  --name ros2_dev \
  -v $(pwd):/workspace \
  -w /workspace \
  osrf/ros:humble-desktop \
  bash
```
4. Inside container

```
source /opt/ros/humble/setup.bash
mkdir -p src
colcon build
```

## Minimal C++ ROS2 Node

1. Create a package skeleton:

```bash
cd src
ros2 pkg create --build-type ament_cmake logger --dependencies rclcpp
```

2. Edit logger/src/logger_node.cpp:

```cpp
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

```

3. Edit CMakeLists.txt adding:
```cmake
# Add executable
add_executable(logger_node src/logger_node.cpp)

# Link against ROS2 libraries
ament_target_dependencies(logger_node rclcpp)

# Install executable so ROS2 can find it
install(TARGETS
  logger_node
  DESTINATION lib/${PROJECT_NAME})
```

4. Build it
```bash
cd /workspace
colcon build --packages-select logger
source install/setup.bash
ros2 run logger logger_node
```

5. Run it
```
root@0876ea219664:/workspace# ros2 run logger logger_node
[INFO] [1757466386.285192634] [logger_node]: Logger node running...
[INFO] [1757466387.280913802] [logger_node]: Logger node running...
[INFO] [1757466388.278142010] [logger_node]: Logger node running...
[INFO] [1757466389.282840428] [logger_node]: Logger node running...
[INFO] [1757466390.282330678] [logger_node]: Logger node running...
```