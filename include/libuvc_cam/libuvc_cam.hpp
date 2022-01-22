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

#ifndef LIBUVC_CAM__LIBUVC_CAM_HPP_
#define LIBUVC_CAM__LIBUVC_CAM_HPP_

extern "C" {
#include <libuvc/libuvc.h>
}

#include <rclcpp/logging.hpp>

#include <memory>
#include <string>
#include <stdexcept>

namespace libuvc_cam
{

// Source: https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
template<typename ... Args>
std::string string_format(const std::string & format, Args ... args)
{
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1;
  if (size_s <= 0) {
    throw std::runtime_error{"Error during formatting."};
  }
  auto size = static_cast<size_t>(size_s);
  auto buf = std::make_unique<char[]>(size);
  std::snprintf(buf.get(), size, format.c_str(), args ...);
  return std::string(buf.get(), buf.get() + size - 1);
}

class UvcCamera
{
public:
  UvcCamera(
    const std::string & vendor_id,
    const std::string & product_id,
    const std::string & ser_num);
  ~UvcCamera();

private:
  uvc_context_t * m_ctx = nullptr;
  uvc_device_t * m_dev = nullptr;
  uvc_device_handle_t * m_handle = nullptr;
};

}  // namespace libuvc_cam

#endif  // LIBUVC_CAM__LIBUVC_CAM_HPP_
