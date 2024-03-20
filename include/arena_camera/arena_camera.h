#ifndef BUILD_ARENA_CAMERA_H
#define BUILD_ARENA_CAMERA_H

#include "Arena/ArenaApi.h"
#include "arena_camera/camera_settings.h"
#include "arena_camera/arena_image.h"
#include <opencv2/opencv.hpp>
#include <future>
#include <iostream>
#include <string>
#include <thread>

class ArenaCamera : public Arena::IImageCallback
{
public:
  explicit ArenaCamera(Arena::IDevice * device, CameraSetting & , std::string & ptp_status);

  ArenaCamera(
    Arena::IDevice * device, std::string & camera_name, std::string & frame_id,
    std::string & pixel_format, uint32_t serial_no, uint32_t fps, uint32_t horizontal_binning,
    uint32_t vertical_binning, std::string & ptp_status);

  ArenaCamera();

  ~ArenaCamera();

  std::thread start_stream();

  void stop_stream();

  void destroy_device(Arena::ISystem * system);

  void acquisition();

  std::shared_ptr<Image>  convert_to_image(Arena::IImage * pImage, const std::string & frame_id);

  using ImageCallbackFunction = std::function<void(std::uint32_t, std::shared_ptr<Image> )>;

  ImageCallbackFunction m_signal_publish_image{};

  void set_on_image_callback(ImageCallbackFunction callback);

  void OnImage(Arena::IImage * pImage)
  {
    m_signal_publish_image(m_cam_idx, convert_to_image(pImage, m_frame_id));
  }

private:
  Arena::IDevice * m_device;

  std::string m_camera_name;

  std::string m_frame_id;

  std::string m_pixel_format;

  uint32_t m_serial_no;

  uint32_t m_fps;

  uint32_t m_cam_idx;

  uint32_t m_horizontal_binning;

  uint32_t m_vertical_binning;

  std::string m_ptp_status_;

  uint32_t m_reached_horizontal_binning;

  uint32_t m_reached_vertical_binning;

  std::shared_future<void> future_;

  Arena::IImage * pImage = nullptr;
};

#endif  // BUILD_ARENA_CAMERA_H