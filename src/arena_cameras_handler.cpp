#include "arena_camera/arena_cameras_handler.h"
#include <algorithm>

ArenaCamerasHandler::ArenaCamerasHandler() {

  m_p_system = Arena::OpenSystem();
  m_p_system->UpdateDevices(100);

}

void ArenaCamerasHandler::create_cameras_from_settings(
    CameraSetting &camera_settings) {
  std::vector<Arena::DeviceInfo> devicesInfos = m_p_system->GetDevices();

  if (devicesInfos.size() == 0) {
    throw std::runtime_error("arena_camera: There is no connected devices.");
  }

  auto it = std::find_if(
      devicesInfos.begin(), devicesInfos.end(),
      [&](Arena::DeviceInfo &d_info) {
        return std::to_string(camera_settings.get_serial_no()) ==
               d_info.SerialNumber().c_str();
      });

  if (it != devicesInfos.end()) {
      m_device = m_p_system->CreateDevice(*it);
      m_cameras = new ArenaCamera(m_device, camera_settings);
      m_device->RegisterImageCallback(m_cameras);

  } else {
    throw std::runtime_error(
        "arena_camera: Wrong device serial no in parameters file.");
  }
}

void ArenaCamerasHandler::set_image_callback(
    ArenaCamera::ImageCallbackFunction callback) {

    this->m_cameras->set_on_image_callback(callback);
}

void ArenaCamerasHandler::start_stream() {
//    this->m_cameras.start_stream();
    this->m_cameras->acquisition();;
}

void ArenaCamerasHandler::stop_stream() {
    this->m_cameras->stop_stream();
}

ArenaCamerasHandler::~ArenaCamerasHandler() {
  std::cout << " ~ArenaCamerasHandler()" << std::endl;

  m_device->DeregisterImageCallback(m_cameras);
  this->m_cameras->destroy_device(m_p_system);
  CloseSystem(m_p_system);

  delete m_cameras;
  delete m_p_system;
  delete m_device;
//  m_cameras.clear();
}
