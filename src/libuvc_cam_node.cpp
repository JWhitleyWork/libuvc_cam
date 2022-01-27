// Copyright 2022 Whitley Software Services
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <libuvc_cam/libuvc_cam_node.hpp>

#include <memory>
#include <string>

using libuvc_cam::FrameFormat;

namespace libuvc_cam
{

UvcCameraNode::UvcCameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("uvc_camera_node", options)
{
  auto vendor_id_param = declare_parameter("vendor_id");
  auto product_id_param = declare_parameter("product_id");
  std::string serial_num = declare_parameter<std::string>("serial_num", "");
  std::string video_format = declare_parameter<std::string>("video_fmt", "");
  int requested_width = declare_parameter("image_width", 0);
  int requested_height = declare_parameter("image_height", 0);
  int requested_frame_rate = declare_parameter("frames_per_second", 0);

  if (vendor_id_param.get_type() == rclcpp::PARAMETER_NOT_SET) {
    throw rclcpp::exceptions::InvalidParameterValueException{"vendor_id is missing."};
  } else if (product_id_param.get_type() == rclcpp::PARAMETER_NOT_SET) {
    throw rclcpp::exceptions::InvalidParameterValueException{"product_id is missing."};
  } else {
    m_camera = std::make_unique<UvcCamera>(
      vendor_id_param.get<std::string>(),
      product_id_param.get<std::string>(),
      serial_num);

    if (video_format.empty() &&
      requested_width == 0 &&
      requested_height == 0 &&
      requested_frame_rate == 0)
    {
      // Use first-available stream
      RCLCPP_INFO(
        get_logger(), "No frame parameters specified. Using first available stream type.");
    } else {
      // Try to find supported stream matching requested parameters
      RCLCPP_INFO(
        get_logger(), "Attempting to acquire stream with specified parameters.");

      if (m_camera->format_is_supported(
          FrameFormat::UNCOMPRESSED,
          requested_width,
          requested_height,
          requested_frame_rate))
      {
      } else {
        RCLCPP_FATAL(
          get_logger(), "Requested format is not supported. "
          "See output below for formats supported by this camera.");
        m_camera->print_supported_formats();
        return;
      }
    }
  }
}

}  // namespace libuvc_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(libuvc_cam::UvcCameraNode)
