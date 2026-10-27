#include "v4l2_camera/gst_uyvy_resizer.hpp"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <sensor_msgs/image_encodings.hpp>

#include <cstdint>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace v4l2_camera
{

struct GstUyvyResizer::Impl
{
  Impl(
    std::uint32_t input_width_value,
    std::uint32_t input_height_value,
    std::uint32_t output_width_value,
    std::uint32_t output_height_value)
  : input_width(input_width_value),
    input_height(input_height_value),
    output_width(output_width_value),
    output_height(output_height_value)
  {
    if (
      input_width == 0U || input_height == 0U ||
      output_width == 0U || output_height == 0U)
    {
      throw std::invalid_argument("Resize dimensions must be greater than zero");
    }

    if ((input_width % 2U) != 0U || (output_width % 2U) != 0U) {
      throw std::invalid_argument("UYVY image width must be even");
    }

    gst_init(nullptr, nullptr);

    std::ostringstream description;
    description
      << "appsrc name=src is-live=true block=true "
      << "format=time do-timestamp=false "
      << "! video/x-raw,format=UYVY"
      << ",width=" << input_width
      << ",height=" << input_height
      << ",framerate=30/1"
      << ",pixel-aspect-ratio=1/1"
      << ",interlace-mode=progressive "
      << "! nvvidconv compute-hw=2 interpolation-method=1 "
      << "! video/x-raw,format=UYVY"
      << ",width=" << output_width
      << ",height=" << output_height
      << ",framerate=30/1"
      << ",pixel-aspect-ratio=1/1"
      << ",interlace-mode=progressive "
      << "! appsink name=sink sync=false emit-signals=false "
      << "max-buffers=1 drop=false";

    GError * error = nullptr;
    pipeline = gst_parse_launch(description.str().c_str(), &error);

    if (error != nullptr) {
      const std::string message = error->message != nullptr ?
        error->message : "unknown GStreamer parse error";
      g_error_free(error);

      if (pipeline != nullptr) {
        gst_object_unref(pipeline);
        pipeline = nullptr;
      }

      throw std::runtime_error(
              "Failed to create GStreamer resize pipeline: " + message);
    }

    if (pipeline == nullptr) {
      throw std::runtime_error("GStreamer resize pipeline is null");
    }

    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");

    if (appsrc == nullptr || appsink == nullptr) {
      shutdown();
      throw std::runtime_error(
              "Failed to locate appsrc or appsink in resize pipeline");
    }

    const GstStateChangeReturn state_result =
      gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (state_result == GST_STATE_CHANGE_FAILURE) {
      shutdown();
      throw std::runtime_error(
              "Failed to set GStreamer resize pipeline to PLAYING");
    }
  }

  ~Impl()
  {
    shutdown();
  }

  void shutdown()
  {
    if (appsrc != nullptr) {
      gst_app_src_end_of_stream(GST_APP_SRC(appsrc));
    }

    if (pipeline != nullptr) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
    }

    if (appsrc != nullptr) {
      gst_object_unref(appsrc);
      appsrc = nullptr;
    }

    if (appsink != nullptr) {
      gst_object_unref(appsink);
      appsink = nullptr;
    }

    if (pipeline != nullptr) {
      gst_object_unref(pipeline);
      pipeline = nullptr;
    }
  }

  std::string read_bus_error() const
  {
    if (pipeline == nullptr) {
      return "pipeline is null";
    }

    GstBus * bus = gst_element_get_bus(pipeline);
    if (bus == nullptr) {
      return "no GStreamer bus message";
    }

    GstMessage * message = gst_bus_pop_filtered(
      bus,
      static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    gst_object_unref(bus);

    if (message == nullptr) {
      return "no GStreamer error message";
    }

    std::string result;

    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      GError * error = nullptr;
      gchar * debug = nullptr;

      gst_message_parse_error(message, &error, &debug);

      if (error != nullptr && error->message != nullptr) {
        result = error->message;
      } else {
        result = "unknown GStreamer error";
      }

      if (debug != nullptr) {
        result += " | ";
        result += debug;
        g_free(debug);
      }

      if (error != nullptr) {
        g_error_free(error);
      }
    } else {
      result = "unexpected end of stream";
    }

    gst_message_unref(message);
    return result;
  }

  sensor_msgs::msg::Image::UniquePtr resize(
    const sensor_msgs::msg::Image & input)
  {
    if (input.encoding != sensor_msgs::image_encodings::YUV422) {
      throw std::runtime_error(
              "GStreamer resizer currently accepts only UYVY/yuv422 input");
    }

    if (input.width != input_width || input.height != input_height) {
      std::ostringstream message;
      message
        << "Unexpected input dimensions: "
        << input.width << "x" << input.height
        << ", expected "
        << input_width << "x" << input_height;
      throw std::runtime_error(message.str());
    }

    const std::uint32_t expected_input_step = input_width * 2U;

    if (input.step != expected_input_step) {
      std::ostringstream message;
      message
        << "Unsupported padded UYVY input step: "
        << input.step << ", expected " << expected_input_step;
      throw std::runtime_error(message.str());
    }

    const std::size_t expected_input_size =
      static_cast<std::size_t>(input.step) *
      static_cast<std::size_t>(input.height);

    if (input.data.size() != expected_input_size) {
      throw std::runtime_error("UYVY input buffer size is inconsistent");
    }

    GstBuffer * buffer =
      gst_buffer_new_allocate(nullptr, input.data.size(), nullptr);

    if (buffer == nullptr) {
      throw std::runtime_error("Failed to allocate GStreamer input buffer");
    }

    const gsize bytes_written = gst_buffer_fill(
      buffer, 0U, input.data.data(), input.data.size());

    if (bytes_written != input.data.size()) {
      gst_buffer_unref(buffer);
      throw std::runtime_error(
              "Failed to copy the complete image into GstBuffer");
    }

    constexpr guint64 frame_duration = GST_SECOND / 30U;

    GST_BUFFER_PTS(buffer) = frame_index * frame_duration;
    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(buffer) = frame_duration;
    ++frame_index;

    const GstFlowReturn push_result =
      gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);

    // gst_app_src_push_buffer() takes ownership of buffer.
    if (push_result != GST_FLOW_OK) {
      throw std::runtime_error(
              "Failed to push frame to GStreamer appsrc: " +
              read_bus_error());
    }

    GstSample * sample = gst_app_sink_try_pull_sample(
      GST_APP_SINK(appsink), GST_SECOND);

    if (sample == nullptr) {
      throw std::runtime_error(
              "Timed out waiting for resized frame: " +
              read_bus_error());
    }

    GstCaps * caps = gst_sample_get_caps(sample);
    GstBuffer * output_buffer = gst_sample_get_buffer(sample);

    if (caps == nullptr || output_buffer == nullptr) {
      gst_sample_unref(sample);
      throw std::runtime_error(
              "GStreamer output sample has no caps or buffer");
    }

    GstVideoInfo video_info;
    gst_video_info_init(&video_info);

    if (!gst_video_info_from_caps(&video_info, caps)) {
      gst_sample_unref(sample);
      throw std::runtime_error(
              "Failed to parse GStreamer output video caps");
    }

    if (
      GST_VIDEO_INFO_FORMAT(&video_info) != GST_VIDEO_FORMAT_UYVY ||
      GST_VIDEO_INFO_WIDTH(&video_info) != static_cast<gint>(output_width) ||
      GST_VIDEO_INFO_HEIGHT(&video_info) != static_cast<gint>(output_height))
    {
      gst_sample_unref(sample);
      throw std::runtime_error(
              "GStreamer produced unexpected output caps");
    }

    auto output = std::make_unique<sensor_msgs::msg::Image>();
    output->header = input.header;
    output->height = output_height;
    output->width = output_width;
    output->encoding = sensor_msgs::image_encodings::YUV422;
    output->is_bigendian = input.is_bigendian;
    output->step = output_width * 2U;
    output->data.resize(
      static_cast<std::size_t>(output->step) *
      static_cast<std::size_t>(output->height));

    GstVideoFrame video_frame{};

    if (!gst_video_frame_map(
        &video_frame, &video_info, output_buffer, GST_MAP_READ))
    {
      gst_sample_unref(sample);
      throw std::runtime_error(
              "Failed to map resized GStreamer output frame");
    }

    const auto * source_data = static_cast<const std::uint8_t *>(
      GST_VIDEO_FRAME_PLANE_DATA(&video_frame, 0));

    const gint source_stride =
      GST_VIDEO_FRAME_PLANE_STRIDE(&video_frame, 0);

    if (
      source_data == nullptr || source_stride < 0 ||
      static_cast<std::uint32_t>(source_stride) < output->step)
    {
      gst_video_frame_unmap(&video_frame);
      gst_sample_unref(sample);
      throw std::runtime_error(
              "Invalid stride in resized GStreamer frame");
    }

    for (std::uint32_t row = 0U; row < output_height; ++row) {
      const std::size_t source_offset =
        static_cast<std::size_t>(row) *
        static_cast<std::size_t>(source_stride);

      const std::size_t destination_offset =
        static_cast<std::size_t>(row) *
        static_cast<std::size_t>(output->step);

      std::memcpy(
        output->data.data() + destination_offset,
        source_data + source_offset,
        output->step);
    }

    gst_video_frame_unmap(&video_frame);
    gst_sample_unref(sample);

    return output;
  }

  std::uint32_t input_width;
  std::uint32_t input_height;
  std::uint32_t output_width;
  std::uint32_t output_height;

  GstElement * pipeline{nullptr};
  GstElement * appsrc{nullptr};
  GstElement * appsink{nullptr};

  guint64 frame_index{0U};
};

GstUyvyResizer::GstUyvyResizer(
  std::uint32_t input_width,
  std::uint32_t input_height,
  std::uint32_t output_width,
  std::uint32_t output_height)
: impl_(std::make_unique<Impl>(
      input_width, input_height, output_width, output_height))
{
}

GstUyvyResizer::~GstUyvyResizer() = default;

sensor_msgs::msg::Image::UniquePtr GstUyvyResizer::resize(
  const sensor_msgs::msg::Image & input)
{
  return impl_->resize(input);
}

}  // namespace v4l2_camera
