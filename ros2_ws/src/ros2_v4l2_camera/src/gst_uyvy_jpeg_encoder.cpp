#include "v4l2_camera/gst_uyvy_jpeg_encoder.hpp"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <sensor_msgs/image_encodings.hpp>

#include <cstdint>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace v4l2_camera
{

struct GstUyvyJpegEncoder::Impl
{
  Impl(
    std::uint32_t input_width_value,
    std::uint32_t input_height_value,
    std::uint32_t output_width_value,
    std::uint32_t output_height_value,
    int quality_value)
  : input_width(input_width_value),
    input_height(input_height_value),
    output_width(output_width_value),
    output_height(output_height_value),
    quality(quality_value)
  {
    if (
      input_width == 0U || input_height == 0U ||
      output_width == 0U || output_height == 0U)
    {
      throw std::invalid_argument(
              "JPEG dimensions must be greater than zero");
    }

    if ((input_width % 2U) != 0U || (output_width % 2U) != 0U) {
      throw std::invalid_argument(
              "UYVY and NV12 image widths must be even");
    }

    if (quality < 0 || quality > 100) {
      throw std::invalid_argument(
              "JPEG quality must be between 0 and 100");
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
      << "output-buffers=8 "
      << "! video/x-raw(memory:NVMM),format=NV12"
      << ",width=" << output_width
      << ",height=" << output_height
      << ",framerate=30/1"
      << ",pixel-aspect-ratio=1/1"
      << ",interlace-mode=progressive "
      << "! nvjpegenc quality=" << quality
      << " Enableperf=false "
      << "! appsink name=sink sync=false emit-signals=false "
      << "max-buffers=1 drop=false";

    GError * error = nullptr;
    pipeline = gst_parse_launch(description.str().c_str(), &error);

    if (error != nullptr) {
      const std::string message =
        error->message != nullptr ?
        error->message :
        "unknown GStreamer parse error";

      g_error_free(error);

      if (pipeline != nullptr) {
        gst_object_unref(pipeline);
        pipeline = nullptr;
      }

      throw std::runtime_error(
              "Failed to create integrated JPEG pipeline: " +
              message);
    }

    if (pipeline == nullptr) {
      throw std::runtime_error(
              "Integrated JPEG pipeline is null");
    }

    appsrc =
      gst_bin_get_by_name(GST_BIN(pipeline), "src");

    appsink =
      gst_bin_get_by_name(GST_BIN(pipeline), "sink");

    if (appsrc == nullptr || appsink == nullptr) {
      shutdown();
      throw std::runtime_error(
              "Failed to locate appsrc or appsink "
              "in integrated JPEG pipeline");
    }

    const GstStateChangeReturn state_result =
      gst_element_set_state(
      pipeline,
      GST_STATE_PLAYING);

    if (state_result == GST_STATE_CHANGE_FAILURE) {
      shutdown();
      throw std::runtime_error(
              "Failed to set integrated JPEG pipeline "
              "to PLAYING");
    }
  }

  ~Impl()
  {
    shutdown();
  }

  void shutdown()
  {
    if (appsrc != nullptr) {
      gst_app_src_end_of_stream(
        GST_APP_SRC(appsrc));
    }

    if (pipeline != nullptr) {
      gst_element_set_state(
        pipeline,
        GST_STATE_NULL);
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

    GstBus * bus =
      gst_element_get_bus(pipeline);

    if (bus == nullptr) {
      return "no GStreamer bus message";
    }

    GstMessage * message =
      gst_bus_pop_filtered(
      bus,
      static_cast<GstMessageType>(
        GST_MESSAGE_ERROR |
        GST_MESSAGE_EOS));

    gst_object_unref(bus);

    if (message == nullptr) {
      return "no GStreamer error message";
    }

    std::string result;

    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      GError * error = nullptr;
      gchar * debug = nullptr;

      gst_message_parse_error(
        message,
        &error,
        &debug);

      if (
        error != nullptr &&
        error->message != nullptr)
      {
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

  sensor_msgs::msg::CompressedImage::UniquePtr encode(
    const sensor_msgs::msg::Image & input)
  {
    if (
      input.encoding !=
      sensor_msgs::image_encodings::YUV422)
    {
      throw std::runtime_error(
              "Integrated JPEG encoder accepts only "
              "UYVY/yuv422 input");
    }

    if (
      input.width != input_width ||
      input.height != input_height)
    {
      std::ostringstream message;
      message
        << "Unexpected JPEG input dimensions: "
        << input.width << "x" << input.height
        << ", expected "
        << input_width << "x" << input_height;

      throw std::runtime_error(message.str());
    }

    const std::uint32_t expected_input_step =
      input_width * 2U;

    if (input.step != expected_input_step) {
      std::ostringstream message;
      message
        << "Unsupported padded UYVY input step: "
        << input.step
        << ", expected "
        << expected_input_step;

      throw std::runtime_error(message.str());
    }

    const std::size_t expected_input_size =
      static_cast<std::size_t>(input.step) *
      static_cast<std::size_t>(input.height);

    if (input.data.size() != expected_input_size) {
      throw std::runtime_error(
              "UYVY input buffer size is inconsistent");
    }

    GstBuffer * buffer =
      gst_buffer_new_allocate(
      nullptr,
      input.data.size(),
      nullptr);

    if (buffer == nullptr) {
      throw std::runtime_error(
              "Failed to allocate JPEG input GstBuffer");
    }

    const gsize bytes_written =
      gst_buffer_fill(
      buffer,
      0U,
      input.data.data(),
      input.data.size());

    if (bytes_written != input.data.size()) {
      gst_buffer_unref(buffer);
      throw std::runtime_error(
              "Failed to copy complete image "
              "into JPEG GstBuffer");
    }

    constexpr guint64 frame_duration =
      GST_SECOND / 30U;

    GST_BUFFER_PTS(buffer) =
      frame_index * frame_duration;

    GST_BUFFER_DTS(buffer) =
      GST_CLOCK_TIME_NONE;

    GST_BUFFER_DURATION(buffer) =
      frame_duration;

    ++frame_index;

    const GstFlowReturn push_result =
      gst_app_src_push_buffer(
      GST_APP_SRC(appsrc),
      buffer);

    // gst_app_src_push_buffer() takes ownership of buffer.
    if (push_result != GST_FLOW_OK) {
      throw std::runtime_error(
              "Failed to push frame to integrated JPEG appsrc: " +
              read_bus_error());
    }

    GstSample * sample =
      gst_app_sink_try_pull_sample(
      GST_APP_SINK(appsink),
      2U * GST_SECOND);

    if (sample == nullptr) {
      throw std::runtime_error(
              "Timed out waiting for integrated JPEG frame: " +
              read_bus_error());
    }

    GstBuffer * output_buffer =
      gst_sample_get_buffer(sample);

    if (output_buffer == nullptr) {
      gst_sample_unref(sample);
      throw std::runtime_error(
              "Integrated JPEG sample has no buffer");
    }

    GstMapInfo map_info{};

    if (!gst_buffer_map(
        output_buffer,
        &map_info,
        GST_MAP_READ))
    {
      gst_sample_unref(sample);
      throw std::runtime_error(
              "Failed to map integrated JPEG buffer");
    }

    auto output =
      std::make_unique<
      sensor_msgs::msg::CompressedImage>();

    output->header = input.header;
    output->format = "jpeg";
    output->data.assign(
      map_info.data,
      map_info.data + map_info.size);

    gst_buffer_unmap(
      output_buffer,
      &map_info);

    gst_sample_unref(sample);

    if (
      output->data.size() < 4U ||
      output->data[0] != 0xFFU ||
      output->data[1] != 0xD8U ||
      output->data[output->data.size() - 2U] != 0xFFU ||
      output->data[output->data.size() - 1U] != 0xD9U)
    {
      throw std::runtime_error(
              "Integrated encoder produced "
              "an invalid JPEG marker sequence");
    }

    return output;
  }

  std::uint32_t input_width;
  std::uint32_t input_height;
  std::uint32_t output_width;
  std::uint32_t output_height;
  int quality;

  GstElement * pipeline{nullptr};
  GstElement * appsrc{nullptr};
  GstElement * appsink{nullptr};

  guint64 frame_index{0U};
};

GstUyvyJpegEncoder::GstUyvyJpegEncoder(
  std::uint32_t input_width,
  std::uint32_t input_height,
  std::uint32_t output_width,
  std::uint32_t output_height,
  int quality)
: impl_(std::make_unique<Impl>(
      input_width,
      input_height,
      output_width,
      output_height,
      quality))
{
}

GstUyvyJpegEncoder::~GstUyvyJpegEncoder() = default;

sensor_msgs::msg::CompressedImage::UniquePtr
GstUyvyJpegEncoder::encode(
  const sensor_msgs::msg::Image & input)
{
  return impl_->encode(input);
}

}  // namespace v4l2_camera
