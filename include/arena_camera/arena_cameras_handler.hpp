#ifndef BUILD_CAMERAS_WRAPPER_H
#define BUILD_CAMERAS_WRAPPER_H

#include "Arena/ArenaApi.h"
#include "arena_camera/arena_camera.hpp"
#include "arena_camera/camera_settings.hpp"

#include <vector>

namespace arena_camera
{
class ArenaCamerasHandler
{
public:
  explicit ArenaCamerasHandler();

  ~ArenaCamerasHandler();

  void create_camera_from_settings(CameraSettings & camera_settings);

  void set_image_callback(ArenaCamera::ImageCallbackFunction callback);

  void start_stream();

  void stop_stream();

  void set_fps(uint32_t fps);

  GenICam_3_3_LUCID::gcstring get_auto_exposure();

  void set_auto_exposure(bool auto_exposure);

  void set_exposure_value(double exposure_value);

  GenICam_3_3_LUCID::gcstring get_auto_gain();

  void set_auto_gain(bool auto_gain);

  void set_gain_value(double gain_value);

  void set_brightness(long brightness);

  void set_enable_rectifying(bool enable_rectifying);

  bool get_enable_rectifying();

  void set_enable_compressing(bool enable_compressing);

  bool get_enable_compressing();

  void set_use_default_device_settings(bool use_default_device_settings);

  bool get_use_default_device_settings();

  void set_ptp_status(bool use_ptp);

  bool ptp_enable();

  bool ptp_disable();

  void set_pixel_format(std::string pixel_format);

  void set_exposure_auto_limit_auto(bool exposure_auto_limit_auto);

  void set_exposure_auto_lower_limit(double exposure_auto_lower_limit);

  void set_exposure_auto_upper_limit(double exposure_auto_upper_limit);

  void set_exposure_damping(double exposure_damping);

  void set_lut_enable(bool lut_enable);

  GenICam_3_3_LUCID::gcstring get_balance_white_auto();

  void set_balance_white_auto(bool balance_white_auto);

  void set_balance_ratio_selector(uint32_t balance_ratio_selector);

  void set_balance_ratio(double balance_ratio);

private:
  ArenaCamera * m_cameras;

  Arena::ISystem * m_p_system;

  Arena::IDevice * m_device;

  bool m_enable_rectifying;
  bool m_enable_compressing;
  bool m_use_default_device_settings;

  std::string m_ptp_status_;

  enum PixelFormat {
    UNKNOWN_FORMAT,
    MONO8,
    MONO16,
    BGR8,
    RGB8,
    BAYER_BGGR8,
    BAYER_GBRG8,
    BAYER_RGGB8,
    BAYER_RGGB16,
  };

  inline std::string PixelFormatToArenaString(PixelFormat pixel_format)
  {
    switch (pixel_format) {
      case PixelFormat::MONO8:
        return "Mono8";
      case PixelFormat::MONO16:
        return "Mono16";
      case PixelFormat::BGR8:
        return "BGR8";
      case PixelFormat::RGB8:
        return "RGB8";
      case PixelFormat::BAYER_BGGR8:
        return "BayerBG8";
      case PixelFormat::BAYER_GBRG8:
        return "BayerGB8";
      case PixelFormat::BAYER_RGGB8:
        return "BayerRG8";
      case PixelFormat::BAYER_RGGB16:
        return "BayerRG16";
      default:
        return "";
    }
  }

  inline PixelFormat ArenaStringToPixelFormat(const std::string & pixel_format_string)
  {
    if (pixel_format_string == "Mono8") {
      return PixelFormat::MONO8;
    }
    if (pixel_format_string == "Mono16") {
      return PixelFormat::MONO16;
    }
    if (pixel_format_string == "BGR8") {
      return PixelFormat::BGR8;
    }
    if (pixel_format_string == "RGB8") {
      return PixelFormat::RGB8;
    }
    if (pixel_format_string == "BayerBG8") {
      return PixelFormat::BAYER_BGGR8;
    }
    if (pixel_format_string == "BayerGB8") {
      return PixelFormat::BAYER_GBRG8;
    }
    if (pixel_format_string == "BayerRG8") {
      return PixelFormat::BAYER_RGGB8;
    }
    if (pixel_format_string == "BayerRG16") {
      return PixelFormat::BAYER_RGGB16;
    }
    return PixelFormat::UNKNOWN_FORMAT;
  }
  inline std::string PixelFormatToRosString(PixelFormat pixel_format)
  {
    switch (pixel_format) {
      case PixelFormat::MONO8:
        return "mono8";
      case PixelFormat::MONO16:
        return "mono16";
      case PixelFormat::BGR8:
        return "bgr8";
      case PixelFormat::RGB8:
        return "rgb8";
      case PixelFormat::BAYER_BGGR8:
        return "bayer_bggr8";
      case PixelFormat::BAYER_GBRG8:
        return "bayer_gbrg8";
      case PixelFormat::BAYER_RGGB8:
        return "bayer_rggb8";
      case PixelFormat::BAYER_RGGB16:
        return "bayer_rggb16";
      default:
        return "";
    }
  }
  inline PixelFormat RosStringToPixelFormat(const std::string & ros_pixel_format_string)
  {
    if (ros_pixel_format_string == "mono8") {
      return PixelFormat::MONO8;
    }
    if (ros_pixel_format_string == "mono16") {
      return PixelFormat::MONO16;
    }
    if (ros_pixel_format_string == "bgr8") {
      return PixelFormat::BGR8;
    }
    if (ros_pixel_format_string == "rgb8") {
      return PixelFormat::RGB8;
    }
    if (ros_pixel_format_string == "bayer_bggr8") {
      return PixelFormat::BAYER_BGGR8;
    }
    if (ros_pixel_format_string == "bayer_gbrg8") {
      return PixelFormat::BAYER_GBRG8;
    }
    if (ros_pixel_format_string == "bayer_rggb8") {
      return PixelFormat::BAYER_RGGB8;
    }
    if (ros_pixel_format_string == "bayer_rggb16") {
      return PixelFormat::BAYER_RGGB16;
    }
    return PixelFormat::UNKNOWN_FORMAT;
  }
};
}  // namespace arena_camera

#endif  // BUILD_CAMERAS_WRAPPER_H