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

#include <libuvc_cam/libuvc_cam.hpp>

#include <cstdio>
#include <stdexcept>
#include <string>

namespace libuvc_cam
{

UvcCamera::UvcCamera(
  const std::string & vendor_id,
  const std::string & product_id,
  const std::string & ser_num)
{
  // Parse vendor ID and product ID
  int vid{}, pid{};

  try {
    vid = std::stoi(vendor_id, 0, 16);
  } catch (const std::invalid_argument & ex) {
    throw std::invalid_argument{"Vendor ID is not a valid hexadecimal value."};
  } catch (const std::out_of_range & ex) {
    throw std::out_of_range{"Vendor ID value is too large."};
  }

  try {
    pid = std::stoi(product_id, 0, 16);
  } catch (const std::invalid_argument & ex) {
    throw std::invalid_argument{"Product ID is not a valid hexadecimal value."};
  } catch (const std::out_of_range & ex) {
    throw std::out_of_range{"Product ID value is too large."};
  }

  // Initialize UVC context
  uvc_error_t res;

  res = uvc_init(&m_ctx, NULL);

  if (res < 0) {
    throw std::runtime_error{
            string_format("Failed to initialize UVC context: %s", uvc_strerror(res))};
  }

  // Initialize UVC device
  if (ser_num.empty()) {
    res = uvc_find_device(m_ctx, &m_dev, vid, pid, NULL);
  } else {
    res = uvc_find_device(m_ctx, &m_dev, vid, pid, ser_num.c_str());
  }

  if (res < 0) {
    throw std::runtime_error{
            string_format(
              "UVC device %04x:%04x with serial number %s was not found: %s",
              vid, pid, ser_num.c_str(), uvc_strerror(res)
            )
    };
  }

  // Open UVC device
  res = uvc_open(m_dev, &m_handle);

  if (res < 0) {
    throw std::runtime_error{
            string_format("UVC device was found but could not be opened: %s", uvc_strerror(res))};
  }
}

UvcCamera::~UvcCamera()
{
  if (m_handle != nullptr) {
    uvc_close(m_handle);
  }

  if (m_dev != nullptr) {
    uvc_unref_device(m_dev);
  }

  if (m_ctx != nullptr) {
    uvc_exit(m_ctx);
  }
}

}  // namespace libuvc_cam
