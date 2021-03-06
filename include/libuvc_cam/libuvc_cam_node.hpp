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

#ifndef LIBUVC_CAM__LIBUVC_CAM_NODE_HPP_
#define LIBUVC_CAM__LIBUVC_CAM_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <libuvc_cam/libuvc_cam.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <memory>

namespace libuvc_cam
{

class UvcCameraNode : public rclcpp::Node
{
public:
  explicit UvcCameraNode(const rclcpp::NodeOptions & options);

private:
  void frame_callback(UvcFrame * frame);

  std::string m_frame{};
  std::unique_ptr<UvcCamera> m_camera{};
  image_transport::Publisher m_image_pub{};
};

}  // namespace libuvc_cam

#endif  // LIBUVC_CAM__LIBUVC_CAM_NODE_HPP_
