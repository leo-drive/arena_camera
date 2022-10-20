#include "arena_camera/arena_cameras_handler.h"

#include <algorithm>

ArenaCamerasHandler::ArenaCamerasHandler()
{
  m_p_system = Arena::OpenSystem();
  m_p_system->UpdateDevices(100);
}

void ArenaCamerasHandler::create_cameras_from_settings(CameraSetting & camera_settings)
{
  std::vector<Arena::DeviceInfo> devicesInfos = m_p_system->GetDevices();

  if (devicesInfos.size() == 0) {
    throw std::runtime_error("arena_camera: There is no connected devices.");
  }

  auto it = std::find_if(devicesInfos.begin(), devicesInfos.end(), [&](Arena::DeviceInfo & d_info) {
    return std::to_string(camera_settings.get_serial_no()) == d_info.SerialNumber().c_str();
  });

  if (it != devicesInfos.end()) {
    m_device = m_p_system->CreateDevice(*it);
    m_cameras = new ArenaCamera(m_device, camera_settings);
    m_device->RegisterImageCallback(m_cameras);

  } else {
    throw std::runtime_error("arena_camera: Wrong device serial no in parameters file.");
  }
}

void ArenaCamerasHandler::set_image_callback(ArenaCamera::ImageCallbackFunction callback)
{
  this->m_cameras->set_on_image_callback(callback);
}

void ArenaCamerasHandler::start_stream() { this->m_cameras->acquisition(); }

void ArenaCamerasHandler::stop_stream() { this->m_cameras->stop_stream(); }

void ArenaCamerasHandler::set_fps(uint32_t fps)
{
  auto node_map = m_device->GetNodeMap();
  auto max_fps = GenApi::CFloatPtr(node_map->GetNode("AcquisitionFrameRate"))->GetMax();
  if (fps > max_fps || fps < 0) {
    Arena::SetNodeValue<bool>(node_map, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(node_map, "AcquisitionFrameRate", max_fps);
  } else {
    Arena::SetNodeValue<bool>(node_map, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(node_map, "AcquisitionFrameRate", static_cast<double>(fps));
  }
}

ArenaCamerasHandler::~ArenaCamerasHandler()
{
  std::cout << " ~ArenaCamerasHandler()" << std::endl;

  this->stop_stream();
  m_device->DeregisterImageCallback(m_cameras);
  this->m_cameras->destroy_device(m_p_system);
  CloseSystem(m_p_system);

  delete m_cameras;
  delete m_p_system;
  delete m_device;
}

GenICam_3_3_LUCID::gcstring ArenaCamerasHandler::get_auto_exposure()
{
  return Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "ExposureAuto");
}

void ArenaCamerasHandler::set_auto_exposure(bool auto_exposure)
{
  GenICam_3_3_LUCID::gcstring exposure_auto = auto_exposure ? "Continuous" : "Off";
  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "ExposureAuto", exposure_auto);
}

void ArenaCamerasHandler::set_exposure_value(float exposure_value)
{
  const auto auto_exposure = this->get_auto_exposure();
  if (auto_exposure == "Off") {
    GenApi::CFloatPtr pExposureTime = m_device->GetNodeMap()->GetNode("ExposureTime");
    try {
      if (exposure_value < pExposureTime->GetMin()) {
        exposure_value = pExposureTime->GetMin();
      } else if (exposure_value > pExposureTime->GetMax()) {
        exposure_value = pExposureTime->GetMax();
      }
      pExposureTime->SetValue(exposure_value);
    } catch (const GenICam::GenericException & e) {
      std::cerr << "Exception occurred during exposure value handling: " << e.GetDescription()
                << std::endl;
    }
  }
}