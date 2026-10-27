#include <rclcpp/rclcpp.hpp>

#include <c2_exposure_sampler/msg/camera_exposure_debug.hpp>
#include <time_sync_msgs/msg/camera_timestamp_debug.hpp>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

uint64_t clock_ns(clockid_t clock_id)
{
  timespec ts{};
  if (::clock_gettime(clock_id, &ts) != 0) {
    return 0;
  }

  return
    static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
    static_cast<uint64_t>(ts.tv_nsec);
}

}  // namespace


class C2ExposureSamplerNode : public rclcpp::Node
{
public:
  using ExposureMsg =
    c2_exposure_sampler::msg::CameraExposureDebug;

  using TimestampMsg =
    time_sync_msgs::msg::CameraTimestampDebug;

  C2ExposureSamplerNode()
  : Node("c2_exposure_sampler_node")
  {
    camera_name_ =
      declare_parameter<std::string>("camera_name", "cam1");

    i2c_device_ =
      declare_parameter<std::string>(
        "i2c_device",
        "/dev/i2c-10");

    i2c_address_ =
      declare_parameter<int>("i2c_address", 0x6d);

    max_attempts_ =
      declare_parameter<int>("max_attempts", 3);

    retry_delay_us_ =
      declare_parameter<int>("retry_delay_us", 1000);

    min_raw_count_ =
      declare_parameter<int>("min_raw_count", 1);

    max_raw_count_ =
      declare_parameter<int>("max_raw_count", 8000);

    queue_warn_depth_ =
      declare_parameter<int>("queue_warn_depth", 5);

    if (max_attempts_ < 1) {
      throw std::runtime_error(
        "max_attempts must be >= 1");
    }

    fd_ = ::open(i2c_device_.c_str(), O_RDWR);

    if (fd_ < 0) {
      throw std::runtime_error(
        "Failed to open " + i2c_device_ +
        ": " + std::strerror(errno));
    }

    auto exposure_qos =
      rclcpp::QoS(rclcpp::KeepLast(100));

    exposure_qos.reliable();
    exposure_qos.durability_volatile();

    publisher_ =
      create_publisher<ExposureMsg>(
        "exposure_debug",
        exposure_qos);

    auto timestamp_qos =
      rclcpp::QoS(rclcpp::KeepLast(100));

    timestamp_qos.reliable();
    timestamp_qos.durability_volatile();

    timestamp_subscription_ =
      create_subscription<TimestampMsg>(
        "timestamp_debug",
        timestamp_qos,
        [this](TimestampMsg::ConstSharedPtr msg) {
          on_timestamp(msg);
        });

    worker_ =
      std::thread(
        &C2ExposureSamplerNode::worker_loop,
        this);

    RCLCPP_INFO(
      get_logger(),
      "C2 frame-triggered exposure sampler started: "
      "camera=%s device=%s address=0x%02x "
      "trigger=timestamp_debug",
      camera_name_.c_str(),
      i2c_device_.c_str(),
      i2c_address_);
  }

  ~C2ExposureSamplerNode() override
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stopping_ = true;
    }

    queue_cv_.notify_all();

    if (worker_.joinable()) {
      worker_.join();
    }

    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  struct FrameTrigger
  {
    std_msgs::msg::Header header;

    uint32_t sequence{0};

    uint64_t raw_v4l2_timestamp_ns{0};

    int64_t current_header_stamp_ns{0};

    uint64_t dqbuf_realtime_after_ns{0};

    uint64_t dqbuf_tsc_ns{0};

    uint32_t queue_depth_before_read{0};
  };


  void on_timestamp(
    const TimestampMsg::ConstSharedPtr & msg)
  {
    static std::atomic<uint64_t> callback_count{0};
    const auto cb = ++callback_count;

    if (cb <= 3 || cb % 100 == 0) {
      const uint64_t callback_realtime_ns =
        clock_ns(CLOCK_REALTIME);

      const uint64_t dqbuf_to_callback_ns =
        callback_realtime_ns >= msg->dqbuf_realtime_after_ns ?
        callback_realtime_ns - msg->dqbuf_realtime_after_ns :
        0ULL;

      RCLCPP_INFO(
        get_logger(),
        "TIMESTAMP_CALLBACK count=%lu frame_sequence=%u "
        "dqbuf_to_callback_ms=%.3f",
        static_cast<unsigned long>(cb),
        msg->sequence,
        static_cast<double>(dqbuf_to_callback_ns) / 1.0e6);
    }

    FrameTrigger trigger;

    trigger.header =
      msg->header;

    trigger.sequence =
      msg->sequence;

    trigger.raw_v4l2_timestamp_ns =
      msg->raw_v4l2_timestamp_ns;

    trigger.current_header_stamp_ns =
      msg->current_header_stamp_ns;

    trigger.dqbuf_realtime_after_ns =
      msg->dqbuf_realtime_after_ns;

    trigger.dqbuf_tsc_ns =
      msg->dqbuf_tsc_ns;

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);

      // Ignore an exact duplicate debug record only.
      if (
        have_last_trigger_ &&
        last_trigger_sequence_ == trigger.sequence &&
        last_trigger_raw_timestamp_ns_ ==
          trigger.raw_v4l2_timestamp_ns)
      {
        return;
      }

      last_trigger_sequence_ =
        trigger.sequence;

      last_trigger_raw_timestamp_ns_ =
        trigger.raw_v4l2_timestamp_ns;

      have_last_trigger_ = true;

      queue_.push_back(trigger);

      const auto queue_depth = queue_.size();

      if (
        queue_warn_depth_ > 0 &&
        queue_depth >
          static_cast<std::size_t>(queue_warn_depth_))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Exposure queue backlog: camera=%s depth=%zu",
          camera_name_.c_str(),
          queue_depth);
      }
    }

    queue_cv_.notify_one();
  }


  void worker_loop()
  {
    while (true) {
      FrameTrigger trigger;

      {
        std::unique_lock<std::mutex> lock(queue_mutex_);

        queue_cv_.wait(
          lock,
          [this]() {
            return stopping_ || !queue_.empty();
          });

        if (stopping_ && queue_.empty()) {
          break;
        }

        trigger =
          queue_.front();

        queue_.pop_front();

        trigger.queue_depth_before_read =
          static_cast<uint32_t>(
            queue_.size() + 1);
      }

      process_frame(trigger);
    }
  }


  bool transfer(
    const uint8_t * wdata,
    uint16_t wlen,
    uint8_t * rdata,
    uint16_t rlen,
    int & error_code)
  {
    i2c_msg msgs[2]{};

    msgs[0].addr =
      static_cast<__u16>(i2c_address_);

    msgs[0].flags =
      0;

    msgs[0].len =
      wlen;

    msgs[0].buf =
      const_cast<uint8_t *>(wdata);

    msgs[1].addr =
      static_cast<__u16>(i2c_address_);

    msgs[1].flags =
      I2C_M_RD;

    msgs[1].len =
      rlen;

    msgs[1].buf =
      rdata;

    i2c_rdwr_ioctl_data data{};

    data.msgs =
      msgs;

    data.nmsgs =
      2;

    const int rc =
      ::ioctl(
        fd_,
        I2C_RDWR,
        &data);

    if (rc < 0) {
      error_code = errno;
      return false;
    }

    error_code = 0;
    return true;
  }


  bool read_current_shutter_once(
    uint32_t & raw_count,
    int & error_code)
  {
    static constexpr std::array<uint8_t, 10>
    request1{
      0x33, 0x47, 0x03, 0x00, 0x00,
      0x00, 0x42, 0x00, 0x80, 0x3f
    };

    std::array<uint8_t, 6> response1{};

    if (
      !transfer(
        request1.data(),
        static_cast<uint16_t>(request1.size()),
        response1.data(),
        static_cast<uint16_t>(response1.size()),
        error_code))
    {
      return false;
    }

    const uint8_t token =
      response1[3];

    std::array<uint8_t, 11> request2{
      0x33,
      0x51,
      0x07,
      token,
      0x00,
      0x00,
      0xec,
      0x01,
      0x00,
      0x00,
      static_cast<uint8_t>(token + 0x78)
    };

    std::array<uint8_t, 101> params{};

    if (
      !transfer(
        request2.data(),
        static_cast<uint16_t>(request2.size()),
        params.data(),
        static_cast<uint16_t>(params.size()),
        error_code))
    {
      return false;
    }

    raw_count =
      static_cast<uint32_t>(params[99]) |
      (
        static_cast<uint32_t>(params[100])
        << 8U
      );

    if (
      raw_count <
        static_cast<uint32_t>(min_raw_count_) ||
      raw_count >
        static_cast<uint32_t>(max_raw_count_))
    {
      error_code = -1;
      return false;
    }

    error_code = 0;

    return true;
  }


  void process_frame(
    const FrameTrigger & trigger)
  {
    ExposureMsg out;

    // IMPORTANT:
    // Header now identifies the CAMERA FRAME that triggered
    // this exposure read, not the status-read midpoint.
    out.header =
      trigger.header;

    out.camera_name =
      camera_name_;

    out.sample_sequence =
      sample_sequence_++;

    out.frame_sequence =
      trigger.sequence;

    out.frame_raw_v4l2_timestamp_ns =
      trigger.raw_v4l2_timestamp_ns;

    out.frame_header_stamp_ns =
      trigger.current_header_stamp_ns;

    out.frame_dqbuf_realtime_after_ns =
      trigger.dqbuf_realtime_after_ns;

    out.frame_dqbuf_tsc_ns =
      trigger.dqbuf_tsc_ns;

    out.queue_depth_before_read =
      trigger.queue_depth_before_read;

    out.sample_realtime_before_ns =
      clock_ns(CLOCK_REALTIME);

    if (
      out.sample_realtime_before_ns >=
      trigger.dqbuf_realtime_after_ns)
    {
      out.dqbuf_to_sample_start_ns =
        out.sample_realtime_before_ns -
        trigger.dqbuf_realtime_after_ns;
    } else {
      out.dqbuf_to_sample_start_ns = 0;
    }

    uint32_t raw_count = 0;
    int error_code = 0;
    uint32_t attempts = 0;
    bool valid = false;

    for (
      int attempt = 1;
      attempt <= max_attempts_;
      ++attempt)
    {
      attempts =
        static_cast<uint32_t>(attempt);

      uint32_t candidate = 0;
      int candidate_error = 0;

      if (
        read_current_shutter_once(
          candidate,
          candidate_error))
      {
        raw_count = candidate;
        error_code = 0;
        valid = true;
        break;
      }

      error_code =
        candidate_error;

      if (attempt < max_attempts_) {
        std::this_thread::sleep_for(
          std::chrono::microseconds(
            retry_delay_us_));
      }
    }

    out.sample_realtime_after_ns =
      clock_ns(CLOCK_REALTIME);

    if (
      out.sample_sequence < 3 ||
      out.sample_sequence % 100 == 0 ||
      !valid ||
      attempts > 1)
    {
      RCLCPP_INFO(
        get_logger(),
        "I2C_READ_DONE frame_sequence=%u valid=%d attempts=%u "
        "error=%d duration_ms=%.3f queue_depth=%u",
        trigger.sequence,
        valid ? 1 : 0,
        attempts,
        error_code,
        static_cast<double>(
          out.sample_realtime_after_ns -
          out.sample_realtime_before_ns) / 1.0e6,
        trigger.queue_depth_before_read);
    }

    out.sample_realtime_mid_ns =
      out.sample_realtime_before_ns +
      (
        out.sample_realtime_after_ns -
        out.sample_realtime_before_ns
      ) / 2ULL;

    out.sample_monotonic_ns =
      clock_ns(CLOCK_MONOTONIC);

    out.raw_count =
      valid ? raw_count : 0;

    out.exposure_time_ns =
      valid ?
      static_cast<uint64_t>(raw_count) *
        12500ULL :
      0ULL;

    out.valid =
      valid;

    out.attempts =
      attempts;

    out.error_code =
      error_code;

    publisher_->publish(out);
  }


  std::string camera_name_;
  std::string i2c_device_;

  int i2c_address_{0x6d};
  int max_attempts_{3};
  int retry_delay_us_{1000};
  int min_raw_count_{1};
  int max_raw_count_{8000};
  int queue_warn_depth_{5};

  int fd_{-1};

  rclcpp::Publisher<ExposureMsg>::SharedPtr
    publisher_;

  rclcpp::Subscription<TimestampMsg>::SharedPtr
    timestamp_subscription_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<FrameTrigger> queue_;

  bool stopping_{false};

  bool have_last_trigger_{false};
  uint32_t last_trigger_sequence_{0};
  uint64_t last_trigger_raw_timestamp_ns_{0};

  std::thread worker_;

  std::atomic<uint64_t> sample_sequence_{0};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<C2ExposureSamplerNode>();

    rclcpp::spin(node);

    node.reset();
  } catch (const std::exception & e) {
    std::fprintf(
      stderr,
      "c2_exposure_sampler fatal: %s\n",
      e.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
