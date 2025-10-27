#include "logger/FastDataLogger.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>

FastDataLogger::FastDataLogger()
: Node("fast_data_logger"),
  log_directory_("./logs"),
  bytes_written_(0),
  entries_logged_(0),
  should_stop_(false),
  start_time_(std::chrono::steady_clock::now())
{
  // Create logs directory
  std::filesystem::create_directories(log_directory_);

  // Create initial log file
  create_new_log_file();

  // Start writer thread
  writer_thread_ = std::thread(&FastDataLogger::writer_thread, this);

  // Stats timer
  stats_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    [this]() {
      auto elapsed = std::chrono::steady_clock::now() - start_time_;
      auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

      if (elapsed_sec > 0) {
        double throughput_mb = (bytes_written_ / 1024.0 / 1024.0) / elapsed_sec;
        double entries_per_sec = entries_logged_ / static_cast<double>(elapsed_sec);

        RCLCPP_INFO(
          this->get_logger(),
          "Stats: %.2f MB written, %.0f entries/sec, %.2f MB/s throughput",
          bytes_written_ / 1024.0 / 1024.0, entries_per_sec, throughput_mb);
      }
    }
  );

  RCLCPP_INFO(this->get_logger(), "Fast data logger initialized");
}

FastDataLogger::~FastDataLogger()
{
  should_stop_ = true;
  queue_cv_.notify_all();

  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }

  if (log_file_.is_open()) {
    log_file_.close();
  }
}

void FastDataLogger::create_new_log_file()
{
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;

  ss << log_directory_ << "/log_"
     << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
     << ".bin";

  current_log_filename_ = ss.str();

  if (log_file_.is_open()) {
    log_file_.close();
  }

  log_file_.open(current_log_filename_, std::ios::binary | std::ios::out);

  if (!log_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", current_log_filename_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Created new log file: %s", current_log_filename_.c_str());
  }
}

void FastDataLogger::log_data(const void * data, size_t size)
{
  auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();

  // Create binary entry
  size_t total_size = sizeof(LogEntry) + size;
  std::vector<uint8_t> entry(total_size);

  LogEntry * log_entry = reinterpret_cast<LogEntry *>(entry.data());

  log_entry->timestamp_ns = timestamp;
  log_entry->message_size = static_cast<uint32_t>(size);

  std::memcpy(entry.data() + sizeof(LogEntry), data, size);

  // Queue for async writing
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    write_queue_.push(std::move(entry));
  }
  queue_cv_.notify_one();

  entries_logged_++;
}

void FastDataLogger::log_string(const std::string & message)
{
  log_data(message.c_str(), message.size());
}

void FastDataLogger::writer_thread()
{
  while (!should_stop_) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait(
      lock, [this] {
        return !write_queue_.empty() || should_stop_;
      });

    while (!write_queue_.empty()) {
      auto entry = std::move(write_queue_.front());
      write_queue_.pop();
      lock.unlock();

      // Write to file
      if (log_file_.is_open()) {
        log_file_.write(reinterpret_cast<const char *>(entry.data()), entry.size());
        log_file_.flush();          // Force immediate write for reliability
        bytes_written_ += entry.size();
      }

      lock.lock();
    }
  }
}
