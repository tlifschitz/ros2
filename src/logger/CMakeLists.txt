cmake_minimum_required(VERSION 3.8)
project(logger)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Add executables
add_executable(logger_node_example example_logger_node.cpp)
add_executable(fast_logger_node_example
  example_fast_logger.cpp
  src/FastDataLogger.cpp
)

# Set include directories
target_include_directories(logger_node_example PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(fast_logger_node_example PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link against ROS2 libraries
ament_target_dependencies(logger_node_example rclcpp)
ament_target_dependencies(fast_logger_node_example rclcpp rcl_interfaces)

# Link pthread for threading support
target_link_libraries(fast_logger_node_example pthread)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip linters that require copyrights/licenses
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Main test suite
  ament_add_gtest(test_logger 
    test/test_logger.cpp
    test/test_fast_data_logger_perf.cpp
    test/test_fast_data_logger_unit.cpp
    src/FastDataLogger.cpp
  )

  target_include_directories(test_logger PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  ament_target_dependencies(test_logger rclcpp rcl_interfaces)
  target_link_libraries(test_logger pthread)
endif()

# Install executables
install(TARGETS
  logger_node_example
  fast_logger_node_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
