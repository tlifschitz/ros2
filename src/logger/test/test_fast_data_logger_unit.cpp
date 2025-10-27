#include <gtest/gtest.h>
#include "logger/FastDataLogger.hpp"
#include <fstream>
#include <filesystem>

class LoggerUnitTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    logger_ = std::make_shared<FastDataLogger>();
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

  std::shared_ptr<FastDataLogger> logger_;
};

TEST_F(LoggerUnitTest, BasicLoggingTest)
{
  const std::string test_message = "Hello, World!";

  logger_->log_string(test_message);

  // Allow time for async writing
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Check if log directory exists
  EXPECT_TRUE(std::filesystem::exists("./logs"));

  // Check if log file was created
  bool found_log_file = false;

  for (const auto & entry : std::filesystem::directory_iterator("./logs")) {
    if (entry.path().extension() == ".bin") {
      found_log_file = true;

      // Check file is not empty
      EXPECT_GT(std::filesystem::file_size(entry.path()), 0);
      break;
    }
  }
  EXPECT_TRUE(found_log_file);
}

TEST_F(LoggerUnitTest, BinaryDataTest)
{
  struct TestStruct
  {
    double value1;
    int32_t value2;
    uint16_t value3;
  };

  TestStruct test_data = {3.14159, 42, 65535};

  logger_->log_data(&test_data, sizeof(test_data));

  // Allow time for async writing
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Verify file exists and has expected minimum size
  bool found_log_file = false;

  for (const auto & entry : std::filesystem::directory_iterator("./logs")) {
    if (entry.path().extension() == ".bin") {
      found_log_file = true;
      size_t file_size = std::filesystem::file_size(entry.path());
      size_t expected_min_size = sizeof(LogEntry) + sizeof(TestStruct);
      EXPECT_GE(file_size, expected_min_size);
      break;
    }
  }
  EXPECT_TRUE(found_log_file);
}
