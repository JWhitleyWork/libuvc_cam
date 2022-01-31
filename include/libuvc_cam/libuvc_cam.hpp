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

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

using NsTimepoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

namespace libuvc_cam
{

static constexpr std::array<uvc_frame_format, 14> UNCOMPRESSED_FORMATS =
{
  UVC_FRAME_FORMAT_UNCOMPRESSED,
  UVC_FRAME_FORMAT_YUYV,
  UVC_FRAME_FORMAT_UYVY,
  UVC_FRAME_FORMAT_RGB,
  UVC_FRAME_FORMAT_BGR,
  UVC_FRAME_FORMAT_GRAY8,
  UVC_FRAME_FORMAT_GRAY16,
  UVC_FRAME_FORMAT_BY8,
  UVC_FRAME_FORMAT_BA81,
  UVC_FRAME_FORMAT_SGRBG8,
  UVC_FRAME_FORMAT_SGBRG8,
  UVC_FRAME_FORMAT_SRGGB8,
  UVC_FRAME_FORMAT_SBGGR8,
  UVC_FRAME_FORMAT_NV12
};

static constexpr std::array<uvc_frame_format, 3> COMPRESSED_FORMATS =
{
  UVC_FRAME_FORMAT_COMPRESSED,
  UVC_FRAME_FORMAT_MJPEG,
  UVC_FRAME_FORMAT_H264
};

enum class StreamFormat
{
  ANY,
  UNCOMPRESSED,
  MJPEG
};

enum class UvcFrameFormat : uint8_t
{
  UNKNOWN = UVC_FRAME_FORMAT_UNKNOWN,
  /** Any supported format */
  ANY = UVC_FRAME_FORMAT_ANY,
  UNCOMPRESSED = UVC_FRAME_FORMAT_UNCOMPRESSED,
  COMPRESSED = UVC_FRAME_FORMAT_COMPRESSED,
  /** YUYV/YUV2/YUV422: YUV encoding with one luminance value per pixel and
   * one UV (chrominance) pair for every two pixels.
   */
  YUYV = UVC_FRAME_FORMAT_YUYV,
  UYVY = UVC_FRAME_FORMAT_UYVY,
  /** 24-bit RGB */
  RGB = UVC_FRAME_FORMAT_RGB,
  BGR = UVC_FRAME_FORMAT_BGR,
  /** Motion-JPEG (or JPEG) encoded images */
  MJPEG = UVC_FRAME_FORMAT_MJPEG,
  H264 = UVC_FRAME_FORMAT_H264,
  /** Greyscale images */
  GRAY8 = UVC_FRAME_FORMAT_GRAY8,
  GRAY16 = UVC_FRAME_FORMAT_GRAY16,
  /* Raw colour mosaic images */
  BY8 = UVC_FRAME_FORMAT_BY8,
  BA81 = UVC_FRAME_FORMAT_BA81,
  SGRBG8 = UVC_FRAME_FORMAT_SGRBG8,
  SGBRG8 = UVC_FRAME_FORMAT_SGBRG8,
  SRGGB8 = UVC_FRAME_FORMAT_SRGGB8,
  SBGGR8 = UVC_FRAME_FORMAT_SBGGR8,
  /** YUV420: NV12 */
  NV12 = UVC_FRAME_FORMAT_NV12
};

class FrameDataContainer
{
public:
  FrameDataContainer();
  FrameDataContainer(void * ptr, const std::size_t sz)
  : pointer_(reinterpret_cast<uint8_t *>(ptr)), size_(sz)
  {}

  uint8_t operator[](const std::size_t idx)
  {
    return pointer_[idx];
  }

  const uint8_t operator[](const std::size_t idx) const
  {
    return pointer_[idx];
  }

  std::size_t size() const
  {
    return size_;
  }

  class iterator
  {
    friend libuvc_cam::FrameDataContainer;

    uint8_t * current_;

    explicit iterator(uint8_t * ptr)
    : current_(ptr)
    {}

    iterator(const iterator & other)
    : iterator(other.current_)
    {}

    iterator(iterator && other) noexcept
    : current_(std::exchange(other.current_, nullptr))
    {}

    iterator & operator=(const iterator & other)
    {
      return *this = iterator(other);
    }

    iterator & operator=(iterator && other) noexcept
    {
      std::swap(current_, other.current_);
      return *this;
    }

    uint8_t & operator*()
    {
      return *current_;
    }

    const uint8_t & operator*() const
    {
      return *current_;
    }

    uint8_t * operator->()
    {
      return current_;
    }

    const uint8_t * operator->() const
    {
      return current_;
    }

    iterator & operator++()
    {
      ++current_;
      return *this;
    }

    iterator operator++(int)
    {
      iterator tmp(*this);
      operator++();
      return tmp;
    }

    iterator & operator--()
    {
      --current_;
      return *this;
    }

    iterator operator--(int)
    {
      iterator tmp(*this);
      operator--();
      return tmp;
    }

    bool operator==(const iterator & other) const
    {
      return current_ == other.current_;
    }

    bool operator!=(const iterator & other) const
    {
      return !(*this == other);
    }
  };

  iterator begin()
  {
    return iterator(pointer_);
  }

  const iterator begin() const
  {
    return iterator(pointer_);
  }

  iterator end()
  {
    return iterator(pointer_ + size_);
  }

  const iterator end() const
  {
    return iterator(pointer_ + size_);
  }

private:
  uint8_t * pointer_;
  std::size_t size_;
};

struct UvcFrame
{
  explicit UvcFrame(uvc_frame * frame);

  // Image data for this frame
  FrameDataContainer data;
  // Width of image in pixels
  uint32_t width;
  // Height of image in pixels
  uint32_t height;
  // Pixel data format
  UvcFrameFormat frame_format;
  // Number of bytes per horizontal line (undefined for compressed format)
  size_t step;
  // Frame number
  uint32_t sequence;
  // Approximate system time when the frame capture was started
  NsTimepoint capture_time;
  // Approximate system time when the frame capture ended
  std::chrono::nanoseconds capture_time_finished;
};

class UvcCamera
{
public:
  UvcCamera(
    const std::string & vendor_id,
    const std::string & product_id,
    const std::string & ser_num);
  ~UvcCamera();

  void register_frame_callback(std::function<void(UvcFrame *)> user_func);
  bool format_is_supported(const StreamFormat fmt, int width, int height, int fps);
  void print_supported_formats();
  void start_streaming();
  void start_streaming_with_format(const StreamFormat fmt, int width, int height, int fps);
  void stop_streaming();

  std::unique_ptr<std::function<void(UvcFrame *)>> m_user_ptr{};

private:
  uvc_context_t * m_ctx = nullptr;
  uvc_device_t * m_dev = nullptr;
  uvc_device_handle_t * m_handle = nullptr;
  uvc_stream_ctrl_t m_ctrl;
};

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

}  // namespace libuvc_cam

#endif  // LIBUVC_CAM__LIBUVC_CAM_HPP_
