#ifndef V4L2_CAMERA__GST_UYVY_JPEG_ENCODER_HPP_
#define V4L2_CAMERA__GST_UYVY_JPEG_ENCODER_HPP_

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <memory>

namespace v4l2_camera
{

class GstUyvyJpegEncoder
{
public:
  GstUyvyJpegEncoder(
    std::uint32_t input_width,
    std::uint32_t input_height,
    std::uint32_t output_width,
    std::uint32_t output_height,
    int quality);

  ~GstUyvyJpegEncoder();

  GstUyvyJpegEncoder(const GstUyvyJpegEncoder &) = delete;
  GstUyvyJpegEncoder & operator=(const GstUyvyJpegEncoder &) = delete;

  sensor_msgs::msg::CompressedImage::UniquePtr encode(
    const sensor_msgs::msg::Image & input);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__GST_UYVY_JPEG_ENCODER_HPP_
