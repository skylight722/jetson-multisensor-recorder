#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/*
 * nvpps_ioctl.h uses clockid_t, so <time.h> must be included first.
 */
#include "nvpps_ros_publisher/nvpps_ioctl.h"

#include <rclcpp/rclcpp.hpp>
#include <time_sync_msgs/msg/nv_pps_correlation.hpp>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{

std::uint64_t timespecToNanoseconds(const timespec & value)
{
  return
    static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
    static_cast<std::uint64_t>(value.tv_nsec);
}

}  // namespace

class NvPpsRosPublisher final : public rclcpp::Node
{
public:
  NvPpsRosPublisher()
  : Node("nvpps_ros_publisher")
  {
    device_path_ = declare_parameter<std::string>(
      "device_path",
      "/dev/nvpps0");

    const auto poll_period_ms = declare_parameter<int>(
      "poll_period_ms",
      20);

    if (poll_period_ms < 1 || poll_period_ms > 1000) {
      throw std::invalid_argument(
        "poll_period_ms must be between 1 and 1000");
    }

    fd_ = ::open(
      device_path_.c_str(),
      O_RDONLY | O_CLOEXEC);

    if (fd_ < 0) {
      throw std::runtime_error(
        "Failed to open " + device_path_ + ": " +
        std::string(std::strerror(errno)));
    }

    auto qos = rclcpp::QoS(
      rclcpp::KeepLast(100));

    qos.reliable();
    qos.durability_volatile();

    publisher_ = create_publisher<
      time_sync_msgs::msg::NvPpsCorrelation>(
      "nvpps",
      qos);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(poll_period_ms),
      std::bind(
        &NvPpsRosPublisher::pollEvent,
        this));

    RCLCPP_INFO(
      get_logger(),
      "NvPPS publisher started: device=%s topic=nvpps poll_period=%ld ms",
      device_path_.c_str(),
      static_cast<long>(poll_period_ms));
  }

  ~NvPpsRosPublisher() override
  {
    RCLCPP_INFO(
      get_logger(),
      "NvPPS publisher stopping: "
      "published=%llu duplicates=%llu ioctl_errors=%llu "
      "event_gaps=%llu event_non_increasing=%llu",
      static_cast<unsigned long long>(published_events_),
      static_cast<unsigned long long>(duplicate_reads_),
      static_cast<unsigned long long>(ioctl_errors_),
      static_cast<unsigned long long>(event_gaps_),
      static_cast<unsigned long long>(event_non_increasing_));

    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  void pollEvent()
  {
    nvpps_timeevent event{};
    std::memset(&event, 0, sizeof(event));

    if (::ioctl(fd_, NVPPS_GETEVENT, &event) != 0) {
      ++ioctl_errors_;

      if (
        ioctl_errors_ <= 5U ||
        ioctl_errors_ % 100U == 0U)
      {
        RCLCPP_WARN(
          get_logger(),
          "NVPPS_GETEVENT failed: %s "
          "(error_count=%llu)",
          std::strerror(errno),
          static_cast<unsigned long long>(
            ioctl_errors_));
      }

      return;
    }

    const std::uint32_t current_event =
      static_cast<std::uint32_t>(event.evt_nb);

    if (
      have_previous_event_ &&
      current_event == previous_event_)
    {
      ++duplicate_reads_;
      return;
    }

    if (have_previous_event_) {
      const std::uint32_t event_delta =
        current_event - previous_event_;

      if (
        event_delta == 0U ||
        event_delta > 0x80000000U)
      {
        ++event_non_increasing_;

        RCLCPP_WARN(
          get_logger(),
          "NvPPS event number did not increase: "
          "previous=%u current=%u",
          previous_event_,
          current_event);
      } else if (event_delta > 1U) {
        const std::uint32_t missing =
          event_delta - 1U;

        event_gaps_ += missing;

        RCLCPP_WARN(
          get_logger(),
          "NvPPS event gap: "
          "previous=%u current=%u missing=%u",
          previous_event_,
          current_event,
          missing);
      }
    }

    timespec realtime_sample{};

    if (
      ::clock_gettime(
        CLOCK_REALTIME,
        &realtime_sample) != 0)
    {
      RCLCPP_WARN(
        get_logger(),
        "CLOCK_REALTIME failed: %s",
        std::strerror(errno));

      return;
    }

    const std::uint64_t host_realtime_ns =
      timespecToNanoseconds(realtime_sample);

    const std::uint64_t tsc_ns =
      static_cast<std::uint64_t>(event.tsc);

    const std::uint64_t ptp_ns =
      static_cast<std::uint64_t>(event.ptp);

    auto message =
      time_sync_msgs::msg::NvPpsCorrelation();

    message.header.stamp.sec =
      static_cast<std::int32_t>(
      ptp_ns / 1000000000ULL);

    message.header.stamp.nanosec =
      static_cast<std::uint32_t>(
      ptp_ns % 1000000000ULL);

    message.header.frame_id = "ptp";

    message.sample_index = published_events_;
    message.event_number = current_event;
    message.host_realtime_ns = host_realtime_ns;
    message.tsc_ns = tsc_ns;
    message.ptp_ns = ptp_ns;

    message.secondary_ptp_ns =
      static_cast<std::uint64_t>(
      event.secondary_ptp);

    message.irq_latency_ns =
      static_cast<std::uint64_t>(
      event.irq_latency);

    message.tsc_resolution_ns =
      static_cast<std::uint64_t>(
      event.tsc_res_ns);

    message.event_mode =
      static_cast<std::uint32_t>(
      event.evt_mode);

    message.tsc_mode =
      static_cast<std::uint32_t>(
      event.tsc_mode);

    message.ptp_minus_tsc_ns =
      static_cast<std::int64_t>(ptp_ns) -
      static_cast<std::int64_t>(tsc_ns);

    publisher_->publish(message);

    previous_event_ = current_event;
    have_previous_event_ = true;
    ++published_events_;

    if (
      published_events_ == 1U ||
      published_events_ % 60U == 0U)
    {
      RCLCPP_INFO(
        get_logger(),
        "NvPPS events published=%llu "
        "latest_event=%u tsc=%llu ptp=%llu",
        static_cast<unsigned long long>(
          published_events_),
        current_event,
        static_cast<unsigned long long>(tsc_ns),
        static_cast<unsigned long long>(ptp_ns));
    }
  }

  std::string device_path_;
  int fd_{-1};

  rclcpp::Publisher<
    time_sync_msgs::msg::NvPpsCorrelation>::SharedPtr
    publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool have_previous_event_{false};
  std::uint32_t previous_event_{
    std::numeric_limits<std::uint32_t>::max()};

  std::uint64_t published_events_{0U};
  std::uint64_t duplicate_reads_{0U};
  std::uint64_t ioctl_errors_{0U};
  std::uint64_t event_gaps_{0U};
  std::uint64_t event_non_increasing_{0U};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<NvPpsRosPublisher>();

    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("nvpps_ros_publisher"),
      "%s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
