#include "arena_camera/arena_camera.hpp"

#include <rclcpp/rclcpp.hpp>

namespace arena_camera
{
ArenaCamera::ArenaCamera(
  Arena::IDevice * device, CameraSettings & camera_setting, std::string & ptp_status)
: m_device(device),
  m_camera_name(camera_setting.camera_name),
  m_frame_id(camera_setting.frame_id),
  m_pixel_format(camera_setting.pixel_format),
  m_serial_no(camera_setting.serial_no),
  m_fps(camera_setting.fps),
  m_horizontal_binning(camera_setting.horizontal_binning),
  m_vertical_binning(camera_setting.vertical_binning),
  m_ptp_status_(ptp_status)
{
  std::cout << "Camera:" << m_cam_idx << " is created." << std::endl;
}

ArenaCamera::ArenaCamera(
  Arena::IDevice * device, std::string & camera_name, std::string & frame_id,
  std::string & pixel_format, uint32_t serial_no, uint32_t fps, uint32_t horizontal_binning,
  uint32_t vertical_binning, std::string & ptp_status_)
: m_device(device),
  m_camera_name(camera_name),
  m_frame_id(frame_id),
  m_pixel_format(pixel_format),
  m_serial_no(serial_no),
  m_fps(fps),
  m_horizontal_binning(horizontal_binning),
  m_vertical_binning(vertical_binning),
  m_ptp_status_(ptp_status_)
{
  std::cout << "Camera:" << m_cam_idx << " is created." << std::endl;
}

std::thread ArenaCamera::start_stream()
{
  return std::thread([=] { this->acquisition(); });
}

void ArenaCamera::acquisition()
{
  auto node_map = m_device->GetNodeMap();
  std::cout << "Camera idx:" << m_cam_idx << " acquisition thread." << std::endl;

  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "AcquisitionMode", "Continuous");

  // Enable stream auto negotiate packet size
  Arena::SetNodeValue<bool>(m_device->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

  // Enable stream packet resend
  Arena::SetNodeValue<bool>(m_device->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

  GenApi::CIntegerPtr pBinningHorizontal = m_device->GetNodeMap()->GetNode("BinningHorizontal");
  if (GenApi::IsWritable(pBinningHorizontal)) {
    size_t binning_x_to_set = m_horizontal_binning;
    if (binning_x_to_set < pBinningHorizontal->GetMin()) {
      binning_x_to_set = pBinningHorizontal->GetMin();
    } else if (binning_x_to_set > pBinningHorizontal->GetMax()) {
      binning_x_to_set = pBinningHorizontal->GetMax();
    }
    pBinningHorizontal->SetValue(binning_x_to_set);
    m_reached_horizontal_binning = pBinningHorizontal->GetValue();
    if (m_reached_horizontal_binning != m_horizontal_binning) {
      RCLCPP_INFO(
        rclcpp::get_logger("ARENA_CAMERA"),
        "Not possible to use hardware based binning for horizontal binning:%d , software binning "
        "will be used.",
        m_horizontal_binning);
    }
  } else {
    std::cout << "Binning horizantal value not readable." << std::endl;
  }

  GenApi::CIntegerPtr pBinningVertical = m_device->GetNodeMap()->GetNode("BinningVertical");
  if (GenApi::IsWritable(pBinningVertical)) {
    size_t binning_y_to_set = m_vertical_binning;
    if (binning_y_to_set < pBinningVertical->GetMin()) {
      binning_y_to_set = pBinningVertical->GetMin();
    } else if (binning_y_to_set > pBinningVertical->GetMax()) {
      binning_y_to_set = pBinningVertical->GetMax();
    }
    pBinningHorizontal->SetValue(binning_y_to_set);
    m_reached_vertical_binning = pBinningVertical->GetValue();
    if (m_reached_vertical_binning != m_vertical_binning) {
      RCLCPP_INFO(
        rclcpp::get_logger("ARENA_CAMERA"),
        "Not possible to use hardware based binning for vertical binning:%d , software binning "
        "will be used.",
        m_horizontal_binning);
    }
  } else {
    std::cout << "Binning vertical value not readable." << std::endl;
  }

  m_device->StartStream();
}

void ArenaCamera::stop_stream()
{
  m_device->StopStream();
}

void ArenaCamera::destroy_device(Arena::ISystem * system)
{
  if (m_device != nullptr) {
    system->DestroyDevice(m_device);
  }
}

void ArenaCamera::set_on_image_callback(ImageCallbackFunction callback)
{
  m_signal_publish_image = std::move(callback);
}

std::shared_ptr<Image> ArenaCamera::convert_to_image(
  Arena::IImage * pImage, const std::string & frame_id)
{
  cv::Mat image_cv =
    cv::Mat(pImage->GetHeight(), pImage->GetWidth(), CV_8UC1, (uint8_t *)pImage->GetData());

  cv::Mat image_bgr(image_cv.rows, image_cv.cols, CV_8UC3);
  cvtColor(image_cv, image_bgr, cv::COLOR_BayerBG2BGR);

  if (
    m_vertical_binning / m_reached_vertical_binning != 1 ||
    m_horizontal_binning / m_reached_horizontal_binning != 1) {
    int ext_vertical_binning = m_vertical_binning / m_reached_vertical_binning;
    int ext_horizontal_binning = m_horizontal_binning / m_reached_horizontal_binning;

    cv::resize(
      image_bgr, image_bgr,
      cv::Size(image_bgr.cols / ext_horizontal_binning, image_bgr.rows / ext_vertical_binning));
  }
  auto image =
    std::make_shared<Image>(image_bgr, pImage->GetTimestampNs(), m_pixel_format, m_ptp_status_);

  return image;
}

ArenaCamera::~ArenaCamera()
{
  std::cout << "Camera:" << m_cam_idx << " ~ArenaCamera()" << std::endl;
  stop_stream();
}
}  // namespace arena_camera