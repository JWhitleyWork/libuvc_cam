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
#include <iostream>
#include <memory>
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

bool UvcCamera::format_is_supported(const FrameFormat fmt, int width, int height, int fps)
{
  if (fmt == FrameFormat::ANY && width == 0 && height == 0 && fps == 0) {
    return true;
  }

  bool fmt_found = false;
  bool width_found = false;
  bool height_found = false;
  bool fps_found = false;

  // Loop through interface format descriptors (DLL)
  const uvc_format_desc_t * format_desc = uvc_get_format_descs(m_handle);
  while (format_desc) {
    fmt_found = (fmt == FrameFormat::ANY);
    width_found = (width == 0);
    height_found = (height == 0);
    fps_found = (fps == 0);

    switch (format_desc->bDescriptorSubtype) {
      case UVC_VS_FORMAT_UNCOMPRESSED:
      case UVC_VS_FORMAT_MJPEG:
      case UVC_VS_FORMAT_FRAME_BASED:
        {
          // Loop through frame format descriptors (DLL)
          const uvc_frame_desc_t * frame_desc = format_desc->frame_descs;
          while (frame_desc) {
            switch (frame_desc->bDescriptorSubtype) {
              case UVC_VS_FRAME_UNCOMPRESSED:
                if (fmt == FrameFormat::UNCOMPRESSED) {
                  fmt_found = true;
                }
                break;
              case UVC_VS_FRAME_MJPEG:
                if (fmt == FrameFormat::MJPEG) {
                  fmt_found = true;
                }
                break;
              default:
                break;
            }

            if (fmt_found) {
              if (width != 0 && static_cast<int>(frame_desc->wWidth) == width) {
                width_found = true;
              }

              if (height != 0 && static_cast<int>(frame_desc->wHeight) == height) {
                height_found = true;
              }

              const auto default_fps =
                static_cast<int32_t>(10000000 / frame_desc->dwDefaultFrameInterval);
              if (default_fps == fps) {
                fps_found = true;
              } else if (frame_desc->intervals) {
                // Check non-default frame interval
                uint32_t * interval_ptr = nullptr;

                for (interval_ptr = frame_desc->intervals; *interval_ptr; ++interval_ptr) {
                  const auto interval_fps = static_cast<int32_t>(10000000 / *interval_ptr);

                  if (interval_fps == fps) {
                    fps_found = true;
                    break;
                  }
                }
              }
            }

            frame_desc = frame_desc->next;
          }
        } break;
      default:
        break;
    }

    if (fmt_found && width_found && height_found && fps_found) {
      break;
    }

    format_desc = format_desc->next;
  }

  return fmt_found && width_found && height_found && fps_found;
}

void UvcCamera::print_supported_formats()
{
  uvc_print_diag(m_handle, stderr);
}

}  // namespace libuvc_cam
