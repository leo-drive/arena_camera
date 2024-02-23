#include "arena_camera/arena_cameras_handler.h"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

ArenaCamerasHandler::ArenaCamerasHandler()
{
  m_p_system = Arena::OpenSystem();
  m_p_system->UpdateDevices(100);
}

void ArenaCamerasHandler::create_camera_from_settings(CameraSetting & camera_settings)
{
  std::vector<Arena::DeviceInfo> devicesInfos = m_p_system->GetDevices();

  if (devicesInfos.size() == 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "There is no connected devices. Please connect a device and try again.");
    throw std::runtime_error("arena_camera: There is no connected devices.");
  }

  auto it = std::find_if(devicesInfos.begin(), devicesInfos.end(), [&](Arena::DeviceInfo & d_info) {
    return std::to_string(camera_settings.get_serial_no()) == d_info.SerialNumber().c_str();
  });

  if (it != devicesInfos.end()) {
    m_device = m_p_system->CreateDevice(*it);

    if(camera_settings.get_use_camera_timestamp()){
      this->set_ptp_status(true);
    }

    m_use_default_device_settings = camera_settings.get_use_default_device_settings();

    // Prepare camera settings
    this->set_fps(camera_settings.get_fps());

//    this->set_pixel_format(camera_settings.get_pixel_format());

    if (!m_use_default_device_settings) {
      this->set_auto_exposure(camera_settings.get_enable_exposure_auto());
      this->set_exposure_value(camera_settings.get_auto_exposure_value());
      this->set_auto_gain(camera_settings.get_enable_exposure_auto());
      this->set_gain_value(camera_settings.get_auto_gain_value());
      this->set_gamma_value(camera_settings.get_gamma_value());
    }

    m_cameras = new ArenaCamera(m_device, camera_settings, m_ptp_status_);
    m_device->RegisterImageCallback(m_cameras);

  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Wrong device serial no in parameters file. Please check the serial no and try again.");
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
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set auto exposure. Using default device settings.");
    return;
  }
  GenICam_3_3_LUCID::gcstring exposure_auto = auto_exposure ? "Continuous" : "Off";
  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "ExposureAuto", exposure_auto);
}

void ArenaCamerasHandler::set_exposure_value(float exposure_value)
{
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set exposure value. Using default device settings.");
    return;
  }

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
  }else{
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set exposure value when auto exposure is enabled.");
  }
}
GenICam_3_3_LUCID::gcstring ArenaCamerasHandler::get_auto_gain()
{
  return Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "GainAuto");
}

void ArenaCamerasHandler::set_auto_gain(bool auto_gain)
{
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set auto gain. Using default device settings.");
    return;
  }
  GenICam_3_3_LUCID::gcstring gain_auto = auto_gain ? "Continuous" : "Off";
  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "GainAuto", gain_auto);
}

void ArenaCamerasHandler::set_gain_value(float gain_value)
{
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value. Using default device settings.");
    return;
  }
  const auto auto_gain = this->get_auto_gain();
  if (auto_gain == "Off") {
    GenApi::CFloatPtr pGain = m_device->GetNodeMap()->GetNode("Gain");
    try {
      if (gain_value < pGain->GetMin()) {
        gain_value = pGain->GetMin();
      } else if (gain_value > pGain->GetMax()) {
        gain_value = pGain->GetMax();
      }
      pGain->SetValue(gain_value);
    } catch (const GenICam::GenericException & e) {
      std::cerr << "Exception occurred during gain value handling: " << e.GetDescription()
                << std::endl;
    }
  }else{
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value when auto gain is enabled.");
  }
}
void ArenaCamerasHandler::set_gamma_value(float gamma_value)
{

  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value. Using default device settings.");
    return;
  }

  try {
    GenApi::CBooleanPtr pGammaEnable = m_device->GetNodeMap()->GetNode("GammaEnable");
    if (GenApi::IsWritable(pGammaEnable)) {
      pGammaEnable->SetValue(true);
    }
    GenApi::CFloatPtr pGamma = m_device->GetNodeMap()->GetNode("Gamma");
    if (pGamma && GenApi::IsWritable(pGamma)) {
      if (pGamma->GetMin() > gamma_value) {
        gamma_value = pGamma->GetMin();
      } else if (pGamma->GetMax() < gamma_value) {
        gamma_value = pGamma->GetMax();
      }
      pGamma->SetValue(gamma_value);
    }
  } catch (const GenICam::GenericException & e) {
    std::cerr << "Exception occurred during gamma value handling: " << e.GetDescription()
              << std::endl;
  }
}
void ArenaCamerasHandler::set_use_default_device_settings(bool use_default_device_settings)
{
  this->m_use_default_device_settings = use_default_device_settings;
}
bool ArenaCamerasHandler::get_use_default_device_settings()
{
  return m_use_default_device_settings;
}


bool ArenaCamerasHandler::ptp_enable()
{
  bool success = false;
  if (m_device != nullptr)
  {
    try
    {
      Arena::SetNodeValue<bool>(m_device->GetNodeMap(),
                                "PtpEnable",
                                true);
      GenICam::gcstring ptpStatus = Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(),
                                                                           "PtpStatus");
      std::cout << "PTPStatus: " << ptpStatus << std::endl;
      m_ptp_status_ = ptpStatus;
      success = true;
    }
    catch (const GenICam::GenericException &e)
    {
      std::cout << "Unable to set 'EnablePtp'. Error: " << e.GetDescription() << std::endl;
    }
  }
  return success;
}


bool ArenaCamerasHandler::ptp_disable() {
  bool success = false;
  if (m_device != nullptr)
  {
    try
    {
      Arena::SetNodeValue<bool>(m_device->GetNodeMap(),
                                "PtpEnable",
                                false);
      GenICam::gcstring ptpStatus = Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(),
                                                                           "PtpStatus");
      std::cout << "PTPStatus: " << ptpStatus << std::endl;
      m_ptp_status_ = ptpStatus;
      success = true;
    }
    catch (const GenICam::GenericException &e)
    {
      std::cout << "Unable to set 'DisablePtp'. Error: " << e.GetDescription() << std::endl;
    }
  }
  return success;
}

void ArenaCamerasHandler::set_ptp_status(bool use_ptp)
{
  if (use_ptp)
  {
    if (ptp_enable())
    {
      RCLCPP_INFO(
          rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
          "Camera Timestamp Enabled (PTP)");
    }
  }
  else
  {
    if (ptp_disable())
    {
      RCLCPP_INFO(
          rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
          "Camera Timestamp Disabled (PTP)");
    }
  }
}

void ArenaCamerasHandler::set_pixel_format(std::string pixel_format)
{
  ArenaCamerasHandler::PixelFormat pixel_format_arena =
      ArenaCamerasHandler::RosStringToPixelFormat(pixel_format);
  try
  {
    GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(),
                                                                                  "PixelFormat");

    GenApi::CEnumerationPtr pPixelFormat = m_device->GetNodeMap()->GetNode("PixelFormat");
    if (GenApi::IsWritable(pPixelFormat))
    {
      std::string str_format = PixelFormatToArenaString(pixel_format_arena);
      std::cout << "SetPixelFormat. Setting pixel format to " << str_format << std::endl;
      Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(),
                                             "PixelFormat",
                                             str_format.c_str());
    }
  }
  catch (const GenICam::GenericException &e)
  {
    std::cout << "An exception while setting target image encoding to '" << PixelFormatToArenaString(pixel_format_arena)
              << "' occurred: " << e.GetDescription() << std::endl;
  }
}

