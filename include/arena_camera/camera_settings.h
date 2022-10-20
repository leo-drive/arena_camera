#ifndef BUILD_CAMERA_SETTINGS_H
#define BUILD_CAMERA_SETTINGS_H

#include <iostream>
#include <string>

class CameraSetting
{
public:
  explicit CameraSetting(
    const std::string & camera_name, const std::string & frame_id, const std::string & pixel_format,
    uint32_t serial_no, uint32_t fps, uint32_t horizontal_binning, uint32_t vertical_binning,
    bool resize_image, const std::string & url_camera_info, bool exposure_auto, float exposure_value)
  : m_camera_name{camera_name},
    m_frame_id{frame_id},
    m_pixel_format{pixel_format},
    m_serial_no{serial_no},
    m_fps{fps},
    m_horizontal_binning{horizontal_binning},
    m_vertical_binning{vertical_binning},
    m_resize_image{resize_image},
    m_url_camera_info{url_camera_info},
    m_auto_exposure_enable{exposure_auto},
    m_auto_exposure_value{exposure_value}
  {
    std::cout << "Camera readed from yaml file. Camera Name:" << m_camera_name
              << " Frame id:" << m_frame_id << " Serial no:" << m_serial_no
              << " Pixel_format:" << m_pixel_format << " FPS:" << m_fps
              << " Flip enable:" << m_resize_image << std::endl;
  }

  std::string get_camera_name() { return m_camera_name; }
  std::string get_frame_id() { return m_frame_id; }
  std::string get_pixel_format() { return m_pixel_format; }
  uint32_t get_serial_no() { return m_serial_no; }
  uint32_t get_fps() { return m_fps; }
  uint32_t get_horizontal_binning() { return m_horizontal_binning; }
  uint32_t get_vertical_binning() { return m_vertical_binning; }
  bool get_resize_image() { return m_resize_image; }
  std::string get_url_camera_info() { return m_url_camera_info; }

  bool get_enable_exposure_auto() { return m_auto_exposure_enable; }
  void set_enable_auto_exposure(bool enable_exposure_auto)
  {
    m_auto_exposure_enable = enable_exposure_auto;
  }

  float get_auto_exposure_value() { return m_auto_exposure_value; }
  void set_auto_exposure_value(float exposure_value) { m_auto_exposure_value = exposure_value; }

private:
  std::string m_url_camera_info;
  std::string m_camera_name;
  std::string m_frame_id;
  std::string m_pixel_format;
  uint32_t m_serial_no;
  uint32_t m_fps;
  uint32_t m_horizontal_binning;
  uint32_t m_vertical_binning;
  bool m_resize_image;
  bool m_auto_exposure_enable;
  float m_auto_exposure_value;  // Only relevant if m_auto_exposure_enable=true
};

#endif  // BUILD_CAMERA_SETTINGS_H
