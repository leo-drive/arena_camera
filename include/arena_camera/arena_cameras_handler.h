#ifndef BUILD_CAMERAS_WRAPPER_H
#define BUILD_CAMERAS_WRAPPER_H

#include "Arena/ArenaApi.h"
#include "arena_camera/arena_camera.h"
#include "arena_camera/camera_settings.h"

#include <vector>

class ArenaCamerasHandler
{
public:
  explicit ArenaCamerasHandler();

  ~ArenaCamerasHandler();

  void create_cameras_from_settings(CameraSetting & camera_settings);

  void set_image_callback(ArenaCamera::ImageCallbackFunction callback);

  void start_stream();

  void stop_stream();

  void set_fps(uint32_t fps);

  GenICam_3_3_LUCID::gcstring get_auto_exposure();

  void set_auto_exposure(bool auto_exposure);

  void set_exposure_value(float exposure_value);

  GenICam_3_3_LUCID::gcstring get_auto_gain();

  void set_auto_gain(bool auto_gain);

  void set_gain_value(float gain_value);

  void set_gamma_value(float gamma_value);

private:
  ArenaCamera * m_cameras;

  Arena::ISystem * m_p_system;

  Arena::IDevice * m_device;
};

#endif  // BUILD_CAMERAS_WRAPPER_H
