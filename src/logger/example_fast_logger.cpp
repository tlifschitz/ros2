#include "rclcpp/rclcpp.hpp"
#include "logger/FastDataLogger.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto logger = std::make_shared<FastDataLogger>();

  // Example: Log some test data
  std::thread test_thread([logger]() {
      for (int i = 0; i < 1000; ++i) {
        std::string test_msg = "Test message " + std::to_string(i);
        logger->log_string(test_msg);

        // Log some binary data
        struct TestData
        {
          double value1;
          int32_t value2;
          float value3;
        } test_data = {i * 1.5, i, static_cast<float>(i * 0.1)};

        logger->log_data(&test_data, sizeof(test_data));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

  rclcpp::spin(logger);

  test_thread.join();
  rclcpp::shutdown();
  return 0;
}
