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

using NsTimepoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

namespace libuvc_cam
{

static NsTimepoint timeval_to_chrono(timeval src)
{
  return NsTimepoint{
    std::chrono::nanoseconds{
      src.tv_sec * 1000000000 + src.tv_usec * 1000}};
}

static std::chrono::nanoseconds timespec_to_chrono(timespec src)
{
  return std::chrono::nanoseconds{
    src.tv_sec * 1000000000 + src.tv_nsec};
}

static void m_frame_callback(uvc_frame_t * frame, void * ptr)
{
  auto cam_instance = reinterpret_cast<UvcCamera *>(ptr);

  if (cam_instance->m_user_ptr) {
    UvcFrame new_frame{frame};
    (*(cam_instance->m_user_ptr))(&new_frame);
  }
}

UvcFrame::UvcFrame(uvc_frame * frame)
: data(frame->data, frame->data_bytes),
  width(frame->width),
  height(frame->height),
  step(frame->step),
  sequence(frame->sequence)
{
  frame_format = static_cast<UvcFrameFormat>(
    static_cast<uint8_t>(frame->frame_format));

  capture_time = timeval_to_chrono(frame->capture_time);
  capture_time_finished = timespec_to_chrono(frame->capture_time_finished);
}

UvcCamera::UvcCamera(
  const std::string & vendor_id,
  const std::string & product_id,
  const std::string & ser_num)
: m_streaming{false}
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
  if (m_streaming) {
    stop_streaming();
  }

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

void UvcCamera::register_frame_callback(std::function<void(UvcFrame *)> user_func)
{
  m_user_ptr = std::make_unique<std::function<void(UvcFrame *)>>(user_func);
}

bool UvcCamera::format_is_supported(const StreamFormat fmt, int width, int height, int fps)
{
  if (fmt == StreamFormat::ANY && width == 0 && height == 0 && fps == 0) {
    return true;
  }

  bool fmt_found = false;
  bool width_found = false;
  bool height_found = false;
  bool fps_found = false;

  // Loop through interface format descriptors (DLL)
  const uvc_format_desc_t * format_desc = uvc_get_format_descs(m_handle);
  while (format_desc) {
    fmt_found = (fmt == StreamFormat::ANY);
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
                if (fmt == StreamFormat::UNCOMPRESSED) {
                  fmt_found = true;
                }
                break;
              case UVC_VS_FRAME_MJPEG:
                if (fmt == StreamFormat::MJPEG) {
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

void UvcCamera::start_streaming()
{
  m_streaming = true;
}

void UvcCamera::start_streaming_with_format(
  const StreamFormat fmt, int width, int height, int fps)
{
  uvc_error_t res;
  uvc_frame_format frame_format = UVC_FRAME_FORMAT_ANY;
  bool format_found = false;
  bool width_found = false;
  bool height_found = false;
  bool fps_found = false;

  if (fmt == StreamFormat::UNCOMPRESSED) {
    const uvc_format_desc_t * format_desc = uvc_get_format_descs(m_handle);

    // Loop through available interface format descriptors
    while (format_desc) {
      if (format_desc->bDescriptorSubtype == UVC_VS_FORMAT_UNCOMPRESSED) {
        const uvc_frame_desc_t * frame_desc = format_desc->frame_descs;

        // Loop through available frame format descriptors
        while (frame_desc) {
          format_found = false;
          width_found = false;
          height_found = false;
          fps_found = false;

          if (width == 0) {
            width = frame_desc->wWidth;
            width_found = true;
          } else if (width == frame_desc->wWidth) {
            width_found = true;
          }

          if (height == 0) {
            height = frame_desc->wHeight;
            height_found = true;
          } else if (height == frame_desc->wHeight) {
            height_found = true;
          }

          if (fps == 0) {
            fps = 10000000 / frame_desc->dwDefaultFrameInterval;
            fps_found = true;
          } else {
            if (fps == static_cast<int>(10000000 / frame_desc->dwDefaultFrameInterval)) {
              fps_found = true;
            } else {
              // Loop through available frame intervals
              if (frame_desc->intervals) {
                // Check non-default frame interval
                uint32_t * interval_ptr = nullptr;

                for (interval_ptr = frame_desc->intervals; *interval_ptr; ++interval_ptr) {
                  const auto interval_fps = static_cast<int32_t>(10000000 / *interval_ptr);

                  if (interval_fps == fps) {
                    fps_found = true;
                    // Break out of intervals loop
                    break;
                  }
                }
              }
            }
          }

          // Test available formats until one is found
          for (const auto & fmt : UNCOMPRESSED_FORMATS) {
            res = uvc_get_stream_ctrl_format_size(
              m_handle, &m_ctrl, fmt, width, height, fps);

            if (res >= 0) {
              frame_format = fmt;
              format_found = true;
              break;
            }
          }

          if (format_found && width_found && height_found && fps_found) {
            // Break out of the frame format descriptor loop
            break;
          }

          frame_desc = frame_desc->next;
        }
      }

      if (format_found && width_found && height_found && fps_found) {
        // Break out of the interface format descriptor loop
        break;
      }

      format_desc = format_desc->next;
    }
  } else if (fmt == StreamFormat::MJPEG) {
    const uvc_format_desc_t * format_desc = uvc_get_format_descs(m_handle);

    // Loop through available interface format descriptors
    while (format_desc) {
      if (format_desc->bDescriptorSubtype == UVC_VS_FORMAT_UNCOMPRESSED) {
        const uvc_frame_desc_t * frame_desc = format_desc->frame_descs;

        // Loop through available frame format descriptors
        while (frame_desc) {
          format_found = false;
          width_found = false;
          height_found = false;
          fps_found = false;

          if (width == 0) {
            width = frame_desc->wWidth;
            width_found = true;
          } else if (width == frame_desc->wWidth) {
            width_found = true;
          }

          if (height == 0) {
            height = frame_desc->wHeight;
            height_found = true;
          } else if (height == frame_desc->wHeight) {
            height_found = true;
          }

          if (fps == 0) {
            fps = 10000000 / frame_desc->dwDefaultFrameInterval;
            fps_found = true;
          } else {
            if (fps == static_cast<int>(10000000 / frame_desc->dwDefaultFrameInterval)) {
              fps_found = true;
            } else {
              // Loop through available frame intervals
              if (frame_desc->intervals) {
                // Check non-default frame interval
                uint32_t * interval_ptr = nullptr;

                for (interval_ptr = frame_desc->intervals; *interval_ptr; ++interval_ptr) {
                  const auto interval_fps = static_cast<int32_t>(10000000 / *interval_ptr);

                  if (interval_fps == fps) {
                    fps_found = true;
                    // Break out of intervals loop
                    break;
                  }
                }
              }
            }
          }

          // Test available formats until one is found
          for (const auto & fmt : COMPRESSED_FORMATS) {
            res = uvc_get_stream_ctrl_format_size(
              m_handle, &m_ctrl, fmt, width, height, fps);

            if (res >= 0) {
              frame_format = fmt;
              format_found = true;
              break;
            }
          }

          if (format_found && width_found && height_found && fps_found) {
            // Break out of the frame format descriptor loop
            break;
          }

          frame_desc = frame_desc->next;
        }
      }

      if (format_found && width_found && height_found && fps_found) {
        // Break out of the interface format descriptor loop
        break;
      }

      format_desc = format_desc->next;
    }
  } else {
    format_found = true;
  }

  if (!(format_found && width_found && height_found && fps_found)) {
    throw std::runtime_error{"start_streaming: A matching stream format could not be found."};
  }

  // Attempt to get a stream control with provided parameters
  res = uvc_get_stream_ctrl_format_size(
    m_handle, &m_ctrl, frame_format, width, height, fps);

  if (res < 0) {
    throw std::runtime_error{
            string_format(
              "Found stream format but couldn't initialize control: %s",
              uvc_strerror(res))};
  }

  // Start streaming
  res = uvc_start_streaming(
    m_handle, &m_ctrl, m_frame_callback, reinterpret_cast<void *>(this), 0);

  if (res < 0) {
    throw std::runtime_error{
            string_format(
              "Got valid stream format but couldn't start streaming: %s",
              uvc_strerror(res))};
  }
}

void UvcCamera::stop_streaming()
{
  uvc_stop_streaming(m_handle);
  m_streaming = false;
}

bool UvcCamera::is_streaming() const
{
  return m_streaming;
}

}  // namespace libuvc_cam
