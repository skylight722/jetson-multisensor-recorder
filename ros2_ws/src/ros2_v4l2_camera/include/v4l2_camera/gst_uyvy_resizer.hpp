#ifndef V4L2_CAMERA__GST_UYVY_RESIZER_HPP_
#define V4L2_CAMERA__GST_UYVY_RESIZER_HPP_

#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <memory>

namespace v4l2_camera
{

class GstUyvyResizer
{
public:
  GstUyvyResizer(
    std::uint32_t input_width,
    std::uint32_t input_height,
    std::uint32_t output_width,
    std::uint32_t output_height);

  ~GstUyvyResizer();

  GstUyvyResizer(const GstUyvyResizer &) = delete;
  GstUyvyResizer & operator=(const GstUyvyResizer &) = delete;

  sensor_msgs::msg::Image::UniquePtr resize(
    const sensor_msgs::msg::Image & input);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__GST_UYVY_RESIZER_HPP_
