#ifndef BUILD_CAMERA_SETTINGS_HPP
#define BUILD_CAMERA_SETTINGS_HPP

#include <iostream>
#include <sstream>
#include <string>

namespace arena_camera
{
class CameraSettings
{
public:
  explicit CameraSettings(
    const std::string & camera_name, const std::string & frame_id, const std::string & pixel_format,
    uint32_t serial_no, uint32_t fps, uint32_t horizontal_binning, uint32_t vertical_binning,
    const std::string & url_camera_info, bool exposure_auto, double exposure_value, bool gain_auto,
    double gain_value, double gamma_value, bool enable_rectifying, bool enable_compressing,
    bool use_default_device_settings, bool use_ptp, long target_brightness,
    bool exposure_auto_limit_auto, double exposure_auto_lower_limit,
    double exposure_auto_upper_limit)

  : camera_name{camera_name},
    frame_id{frame_id},
    pixel_format{pixel_format},
    serial_no{serial_no},
    fps{fps},
    horizontal_binning{horizontal_binning},
    vertical_binning{vertical_binning},
    url_camera_info{url_camera_info},
    auto_exposure_enable{exposure_auto},
    exposure_value{exposure_value},
    gain_auto_enable{gain_auto},
    gain_value{gain_value},
    gamma_value{gamma_value},
    enable_rectifying{enable_rectifying},
    enable_compressing{enable_compressing},
    use_default_device_settings{use_default_device_settings},
    use_ptp{use_ptp},
    target_brightness{target_brightness},
    exposure_auto_limit_auto{exposure_auto_limit_auto},
    exposure_auto_lower_limit{exposure_auto_lower_limit},
    exposure_auto_upper_limit{exposure_auto_upper_limit}
  {
    std::stringstream output;

    output << "Camera Info:" << std::endl
           << "  Name: " << camera_name << std::endl
           << "  Frame ID: " << frame_id << std::endl
           << "  Pixel Format: " << pixel_format << std::endl
           << "  Serial Number: " << serial_no << std::endl
           << "  FPS: " << fps << std::endl
           << "  Horizontal Binning: " << horizontal_binning << std::endl
           << "  Vertical Binning: " << vertical_binning << std::endl
           << "  Camera Info URL: " << url_camera_info << std::endl
           << "  Exposure Auto: " << (exposure_auto ? "On" : "Off") << std::endl
           << "  Exposure Value: " << exposure_value << std::endl
           << "  Gain Auto: " << (gain_auto ? "On" : "Off") << std::endl
           << "  Gain Value: " << gain_value << std::endl
           << "  Gamma Value: " << gamma_value << std::endl
           << "  Rectification Enabled: " << (enable_rectifying ? "Yes" : "No") << std::endl
           << "  Compression Enabled: " << (enable_compressing ? "Yes" : "No") << std::endl
           << "  Use Default Device Settings: " << (use_default_device_settings ? "Yes" : "No")
           << std::endl
           << "  Use PTP: " << (use_ptp ? "Yes" : "No") << std::endl
           << "  Target Brightness: " << target_brightness << std::endl
           << "  Exposure Auto Limit Auto: " << (exposure_auto_limit_auto ? "Yes" : "No")
           << std::endl
           << "  Exposure Auto Lower Limit: " << exposure_auto_lower_limit << std::endl
           << "  Exposure Auto Upper Limit: " << exposure_auto_upper_limit << std::endl;

    std::cout << output.str();
  }

  std::string url_camera_info;
  std::string camera_name;
  std::string frame_id;
  std::string pixel_format;
  uint32_t serial_no;
  uint32_t fps;
  uint32_t horizontal_binning;
  uint32_t vertical_binning;
  bool auto_exposure_enable;
  double exposure_value;
  bool gain_auto_enable;
  double gain_value;
  double gamma_value;
  bool enable_rectifying;
  bool enable_compressing;
  bool use_default_device_settings;
  bool use_ptp;
  long target_brightness;
  bool exposure_auto_limit_auto;
  double exposure_auto_lower_limit;
  double exposure_auto_upper_limit;
};
}  // namespace arena_camera

#endif  // BUILD_CAMERA_SETTINGS_HPP