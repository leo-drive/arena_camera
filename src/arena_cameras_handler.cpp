#include "arena_camera/arena_cameras_handler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>

namespace arena_camera
{
ArenaCamerasHandler::ArenaCamerasHandler()
{
  m_p_system = Arena::OpenSystem();
  m_p_system->UpdateDevices(100);
}

void ArenaCamerasHandler::create_camera_from_settings(CameraSettings & camera_settings)
{
  std::vector<Arena::DeviceInfo> devicesInfos = m_p_system->GetDevices();

  if (devicesInfos.size() == 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "There is no connected devices. Please connect a device and try again.");
    throw std::runtime_error("arena_camera: There is no connected devices.");
  }

  auto it = std::find_if(devicesInfos.begin(), devicesInfos.end(), [&](Arena::DeviceInfo & d_info) {
    return std::to_string(camera_settings.serial_no) == d_info.SerialNumber().c_str();
  });

  if (it != devicesInfos.end()) {
    m_device = m_p_system->CreateDevice(*it);
    if (camera_settings.use_ptp) {
      this->set_ptp_status(true);
    }
    m_enable_rectifying = camera_settings.enable_rectifying;
    m_enable_compressing = camera_settings.enable_compressing;
    m_use_default_device_settings = camera_settings.use_default_device_settings;
    // Prepare camera settings
    this->set_fps(camera_settings.fps);
    //    this->set_pixel_format(camera_settings.get_pixel_format());

    if (!m_use_default_device_settings) {
      this->set_auto_exposure(camera_settings.auto_exposure_enable);
      this->set_exposure_value(camera_settings.exposure_value);
      this->set_auto_gain(camera_settings.gain_auto_enable);
      this->set_gain_value(camera_settings.gain_value);
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

void ArenaCamerasHandler::start_stream()
{
  this->m_cameras->acquisition();
}

void ArenaCamerasHandler::stop_stream()
{
  this->m_cameras->stop_stream();
}

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
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set auto exposure. Using default device settings.");
    return;
  }
  GenICam_3_3_LUCID::gcstring exposure_auto = auto_exposure ? "Continuous" : "Off";
  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "ExposureAuto", exposure_auto);
}

void ArenaCamerasHandler::set_exposure_value(double exposure_value)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set exposure value. Using default device settings.");
    return;
  }

  const auto auto_exposure = this->get_auto_exposure();

  // If auto exposure is off, adjust exposure value manually
  GenApi::CFloatPtr pExposureTime = m_device->GetNodeMap()->GetNode("ExposureTime");
  double min_exposure = pExposureTime->GetMin();
  double max_exposure = pExposureTime->GetMax();

  RCLCPP_INFO(
    rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "Allowed exposure value range: [%f, %f]",
    min_exposure, max_exposure);

  if (auto_exposure == "Off") {
    try {
      // Ensure exposure value is within valid range
      if (exposure_value < min_exposure) {
        exposure_value = min_exposure;
        RCLCPP_WARN(
          rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
          "Exposure value below minimum (%f). Adjusted to minimum value.", min_exposure);
      } else if (exposure_value > max_exposure) {
        exposure_value = max_exposure;
        RCLCPP_WARN(
          rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
          "Exposure value above maximum (%f). Adjusted to maximum value.", max_exposure);
      }

      // Set exposure value
      pExposureTime->SetValue(exposure_value);
      RCLCPP_INFO(
        rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "Exposure value set to %f", exposure_value);
    } catch (const GenICam::GenericException & e) {
      // Handle exceptions during exposure value handling
      std::cerr << "Exception occurred during exposure value handling: " << e.GetDescription()
                << std::endl;
      RCLCPP_ERROR(
        rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
        "Exception occurred during exposure value handling: %s", e.GetDescription());
    }
  } else {
    // If auto exposure is enabled, cannot manually set exposure value
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
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set auto gain. Using default device settings.");
    return;
  }
  GenICam_3_3_LUCID::gcstring gain_auto = auto_gain ? "Continuous" : "Off";
  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "GainAuto", gain_auto);
}

void ArenaCamerasHandler::set_gain_value(double gain_value)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value. Using default device settings.");
    return;
  }
  const auto auto_gain = this->get_auto_gain();

  // If auto gain is off, adjust gain value manually
  GenApi::CFloatPtr pGain = m_device->GetNodeMap()->GetNode("Gain");
  double min_gain = pGain->GetMin();
  double max_gain = pGain->GetMax();
  RCLCPP_INFO(
    rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "Allowed gain value range: [%f, %f]", min_gain,
    max_gain);

  if (auto_gain == "Off") {
    try {
      gain_value = std::clamp(gain_value, min_gain, max_gain);

      pGain->SetValue(gain_value);
      RCLCPP_INFO(rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "Gain value set to %f", gain_value);
    } catch (const GenICam::GenericException & e) {
      // Handle exceptions during gain value handling
      std::cerr << "Exception occurred during gain value handling: " << e.GetDescription()
                << std::endl;
      RCLCPP_ERROR(
        rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
        "Exception occurred during gain value handling: %s", e.GetDescription());
    }
  } else {
    // If auto gain is enabled, cannot manually set gain value
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value when auto gain is enabled.");
  }
}

void ArenaCamerasHandler::set_brightness(long brightness)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value. Using default device settings.");
    return;
  }

  // If auto gain is off, adjust gain value manually
  GenApi::CIntegerPtr pTargetBrightness = m_device->GetNodeMap()->GetNode("TargetBrightness");

  if (!pTargetBrightness) {
    RCLCPP_WARN(rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "TargetBrightness is not supported");
    return;
  }

  long brightness_min = pTargetBrightness->GetMin();
  long brightness_max = pTargetBrightness->GetMax();
  RCLCPP_INFO(
    rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "Allowed brightness value range: [%ld, %ld]",
    brightness_min, brightness_max);

  try {
    brightness = std::clamp(brightness, brightness_min, brightness_max);

    pTargetBrightness->SetValue(brightness);
    RCLCPP_INFO(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "TargetBrightness set to %ld", brightness);
  } catch (const GenICam::GenericException & e) {
    // Handle exceptions during gain value handling
    std::cerr << "Exception occurred during TargetBrightness handling: " << e.GetDescription()
              << std::endl;
    RCLCPP_ERROR(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Exception occurred during TargetBrightness handling: %s", e.GetDescription());
  }
}

void ArenaCamerasHandler::set_enable_rectifying(bool enable_rectifying)
{
  this->m_enable_rectifying = enable_rectifying;
}
bool ArenaCamerasHandler::get_enable_rectifying()
{
  return m_enable_rectifying;
}
void ArenaCamerasHandler::set_enable_compressing(bool enable_compressing)
{
  this->m_enable_compressing = enable_compressing;
}
bool ArenaCamerasHandler::get_enable_compressing()
{
  return m_enable_compressing;
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
  if (m_device != nullptr) {
    try {
      Arena::SetNodeValue<bool>(m_device->GetNodeMap(), "PtpEnable", true);
      GenICam::gcstring ptpStatus =
        Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "PtpStatus");
      std::cout << "PTPStatus: " << ptpStatus << std::endl;
      m_ptp_status_ = ptpStatus;
      int64_t PtpOffsetFromMaster =
        Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "PtpOffsetFromMaster");
      std::cout << "Ptp Offset From Master: " << PtpOffsetFromMaster << std::endl;

      success = true;
    } catch (const GenICam::GenericException & e) {
      std::cout << "Unable to set 'EnablePtp'. Error: " << e.GetDescription() << std::endl;
    }
  }
  return success;
}

bool ArenaCamerasHandler::ptp_disable()
{
  bool success = false;
  if (m_device != nullptr) {
    try {
      Arena::SetNodeValue<bool>(m_device->GetNodeMap(), "PtpEnable", false);
      GenICam::gcstring ptpStatus =
        Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "PtpStatus");
      std::cout << "PTPStatus: " << ptpStatus << std::endl;
      m_ptp_status_ = ptpStatus;
      success = true;
    } catch (const GenICam::GenericException & e) {
      std::cout << "Unable to set 'DisablePtp'. Error: " << e.GetDescription() << std::endl;
    }
  }
  return success;
}

void ArenaCamerasHandler::set_ptp_status(bool use_ptp)
{
  if (use_ptp) {
    if (ptp_enable()) {
      RCLCPP_INFO(rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "Camera Timestamp Enabled (PTP)");
    }
  } else {
    if (ptp_disable()) {
      RCLCPP_INFO(rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "Camera Timestamp Disabled (PTP)");
    }
  }
}

void ArenaCamerasHandler::set_pixel_format(std::string pixel_format)
{
  ArenaCamerasHandler::PixelFormat pixel_format_arena =
    ArenaCamerasHandler::RosStringToPixelFormat(pixel_format);
  try {
    GenICam::gcstring pixelFormatInitial =
      Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "PixelFormat");

    GenApi::CEnumerationPtr pPixelFormat = m_device->GetNodeMap()->GetNode("PixelFormat");
    if (GenApi::IsWritable(pPixelFormat)) {
      std::string str_format = PixelFormatToArenaString(pixel_format_arena);
      std::cout << "SetPixelFormat. Setting pixel format to " << str_format << std::endl;
      Arena::SetNodeValue<GenICam::gcstring>(
        m_device->GetNodeMap(), "PixelFormat", str_format.c_str());
    }
  } catch (const GenICam::GenericException & e) {
    std::cout << "An exception while setting target image encoding to '"
              << PixelFormatToArenaString(pixel_format_arena)
              << "' occurred: " << e.GetDescription() << std::endl;
  }
}

void ArenaCamerasHandler::set_exposure_auto_limit_auto(bool exposure_auto_limit_auto)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set exposure auto limit. Using default device settings.");
    return;
  }
  GenICam_3_3_LUCID::gcstring exposure_auto_limit_auto_str =
    exposure_auto_limit_auto ? "Continuous" : "Off";

  Arena::SetNodeValue<GenICam::gcstring>(
    m_device->GetNodeMap(), "ExposureAutoLimitAuto", exposure_auto_limit_auto_str);
}

void ArenaCamerasHandler::set_exposure_auto_lower_limit(double exposure_auto_lower_limit)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set exposure auto lower limit. Using default device settings.");
    return;
  }

  // Check if exposure auto lower limit is supported
  GenApi::CFloatPtr pExposureAutoLowerLimit =
    m_device->GetNodeMap()->GetNode("ExposureAutoLowerLimit");
  if (!pExposureAutoLowerLimit) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "ExposureAutoLowerLimit is not supported");
    return;
  }

  double exposure_auto_lower_limit_min = pExposureAutoLowerLimit->GetMin();
  double exposure_auto_lower_limit_max = pExposureAutoLowerLimit->GetMax();
  RCLCPP_INFO(
    rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
    "Allowed exposure auto lower limit range: [%lf, %lf]", exposure_auto_lower_limit_min,
    exposure_auto_lower_limit_max);

  try {
    exposure_auto_lower_limit = std::clamp(
      exposure_auto_lower_limit, exposure_auto_lower_limit_min, exposure_auto_lower_limit_max);

    pExposureAutoLowerLimit->SetValue(exposure_auto_lower_limit);
    RCLCPP_INFO(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "ExposureAutoLowerLimit set to %lf",
      exposure_auto_lower_limit);
  } catch (const GenICam::GenericException & e) {
    // Handle exceptions during exposure auto lower limit handling
    std::cerr << "Exception occurred during ExposureAutoLowerLimit handling: " << e.GetDescription()
              << std::endl;
    RCLCPP_ERROR(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Exception occurred during ExposureAutoLowerLimit handling: %s", e.GetDescription());
  }
}

void ArenaCamerasHandler::set_exposure_auto_upper_limit(double exposure_auto_upper_limit)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set exposure auto upper limit. Using default device settings.");
    return;
  }

  // Check if exposure auto upper limit is supported
  GenApi::CFloatPtr pExposureAutoUpperLimit =
    m_device->GetNodeMap()->GetNode("ExposureAutoUpperLimit");
  if (!pExposureAutoUpperLimit) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "ExposureAutoUpperLimit is not supported");
    return;
  }

  double exposure_auto_upper_limit_min = pExposureAutoUpperLimit->GetMin();
  double exposure_auto_upper_limit_max = pExposureAutoUpperLimit->GetMax();
  RCLCPP_INFO(
    rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
    "Allowed exposure auto upper limit range: [%lf, %lf]", exposure_auto_upper_limit_min,
    exposure_auto_upper_limit_max);

  try {
    exposure_auto_upper_limit = std::clamp(
      exposure_auto_upper_limit, exposure_auto_upper_limit_min, exposure_auto_upper_limit_max);

    pExposureAutoUpperLimit->SetValue(exposure_auto_upper_limit);
    RCLCPP_INFO(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "ExposureAutoUpperLimit set to %lf",
      exposure_auto_upper_limit);
  } catch (const GenICam::GenericException & e) {
    // Handle exceptions during exposure auto upper limit handling
    std::cerr << "Exception occurred during ExposureAutoUpperLimit handling: " << e.GetDescription()
              << std::endl;
    RCLCPP_ERROR(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Exception occurred during ExposureAutoUpperLimit handling: %s", e.GetDescription());
  }
}

void ArenaCamerasHandler::set_exposure_damping(double exposure_damping)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set exposure damping. Using default device settings.");
    return;
  }

  // Check if exposure damping is supported
  GenApi::CFloatPtr pExposureDamping = m_device->GetNodeMap()->GetNode("ExposureAutoDamping");
  if (!pExposureDamping) {
    RCLCPP_WARN(rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "ExposureDamping is not supported");
    return;
  }

  double exposure_damping_min = pExposureDamping->GetMin();
  double exposure_damping_max = pExposureDamping->GetMax();
  RCLCPP_INFO(
    rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "Allowed exposure damping range: [%lf, %lf]",
    exposure_damping_min, exposure_damping_max);

  try {
    exposure_damping = std::clamp(exposure_damping, exposure_damping_min, exposure_damping_max);

    pExposureDamping->SetValue(exposure_damping);
    RCLCPP_INFO(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "ExposureDamping set to %lf", exposure_damping);
  } catch (const GenICam::GenericException & e) {
    // Handle exceptions during exposure damping handling
    std::cerr << "Exception occurred during ExposureDamping handling: " << e.GetDescription()
              << std::endl;
    RCLCPP_ERROR(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Exception occurred during ExposureDamping handling: %s", e.GetDescription());
  }
}

void ArenaCamerasHandler::set_lut_enable(bool lut_enable)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set Look-Up Table (LUT). Using default device settings.");
    return;
  }
  GenICam_3_3_LUCID::gcstring lut_setting = lut_enable ? "True" : "False" ;
  Arena::SetNodeValue<bool>(m_device->GetNodeMap(), "LUTEnable", lut_setting);
}

GenICam_3_3_LUCID::gcstring ArenaCamerasHandler::get_balance_white_auto()
{
  return Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "BalanceWhiteAuto");
}

void ArenaCamerasHandler::set_balance_white_auto(bool balance_white_auto)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set balance white auto. Using default device settings.");
    return;
  }
  GenICam_3_3_LUCID::gcstring balance_white_auto_str =
    balance_white_auto ? "Continuous" : "Off";

  Arena::SetNodeValue<GenICam::gcstring>(
    m_device->GetNodeMap(), "BalanceWhiteAuto", balance_white_auto_str);
}

void ArenaCamerasHandler::set_balance_ratio_selector(uint32_t balance_ratio_selector)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set balance ratio selector. Using default device settings.");
    return;
  }

  GenICam_3_3_LUCID::gcstring balance_ratio_selector_str;

  switch (balance_ratio_selector) {
    case 0:
      balance_ratio_selector_str = "Red";
      Arena::SetNodeValue<GenICam::gcstring>(
        m_device->GetNodeMap(), "BalanceRatioSelector", balance_ratio_selector_str);
      break;
    case 1:
      balance_ratio_selector_str = "Green";
      Arena::SetNodeValue<GenICam::gcstring>(
        m_device->GetNodeMap(), "BalanceRatioSelector", balance_ratio_selector_str);
      break;
    case 2:
      balance_ratio_selector_str = "Blue";
      Arena::SetNodeValue<GenICam::gcstring>(
        m_device->GetNodeMap(), "BalanceRatioSelector", balance_ratio_selector_str);
      break;
    default:
      RCLCPP_WARN(
        rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
        "Invalid balance ratio selector. Please select from 0, 1, 2.");
      return;
  }
}

void ArenaCamerasHandler::set_balance_ratio(double balance_ratio)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set balance ratio. Using default device settings.");
    return;
  }

  // Get auto balance mode
  const auto auto_balance = this->get_balance_white_auto();

  // If auto balance is off, adjust balance ratio manually
  GenApi::CFloatPtr pBalanceRatio = m_device->GetNodeMap()->GetNode("BalanceRatio");
  double min_balance_ratio = pBalanceRatio->GetMin();
  double max_balance_ratio = pBalanceRatio->GetMax();
  RCLCPP_INFO(
    rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "Allowed balance ratio range: [%f, %f]", min_balance_ratio,
    max_balance_ratio);

  if (auto_balance == "Off") {
    try {
      balance_ratio = std::clamp(balance_ratio, min_balance_ratio, max_balance_ratio);

      pBalanceRatio->SetValue(balance_ratio);
      RCLCPP_INFO(rclcpp::get_logger("ARENA_CAMERA_HANDLER"), "Balance ratio set to %f", balance_ratio);
    } catch (const GenICam::GenericException & e) {
      // Handle exceptions during balance ratio handling
      std::cerr << "Exception occurred during balance ratio handling: " << e.GetDescription()
                << std::endl;
      RCLCPP_ERROR(
        rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
        "Exception occurred during balance ratio handling: %s", e.GetDescription());
    }
  } else {
    // If auto balance is enabled, cannot manually set balance ratio
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set balance ratio when auto balance is enabled.");
  }
}

}  // namespace arena_camera