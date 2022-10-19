#include "arena_camera/arena_camera.h"

ArenaCamera::ArenaCamera(Arena::IDevice * device, CameraSetting & camera_setting)
: m_device(device),
  m_camera_name(camera_setting.get_camera_name()),
  m_frame_id(camera_setting.get_frame_id()),
  m_pixel_format(camera_setting.get_pixel_format()),
  m_serial_no(camera_setting.get_serial_no()),
  m_fps(camera_setting.get_fps()),
  m_horizontal_binning(camera_setting.get_horizontal_binning()),
  m_vertical_binning(camera_setting.get_vertical_binning()),
  m_resize_image(camera_setting.get_resize_image()),
  m_continue_acquiring(true)
{
  std::cout << "Camera:" << m_cam_idx << " is created." << std::endl;
}

ArenaCamera::ArenaCamera(
  Arena::IDevice * device, std::string & camera_name, std::string & frame_id,
  std::string & pixel_format, uint32_t serial_no, uint32_t fps, uint32_t horizontal_binning,
  uint32_t vertical_binning, bool resize_image)
: m_device(device),
  m_camera_name(camera_name),
  m_frame_id(frame_id),
  m_pixel_format(pixel_format),
  m_serial_no(serial_no),
  m_fps(fps),
  m_horizontal_binning(horizontal_binning),
  m_vertical_binning(vertical_binning),
  m_resize_image(resize_image)
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

  // enable stream auto negotiate packet size
  Arena::SetNodeValue<bool>(m_device->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

  //     enable stream packet resend
  Arena::SetNodeValue<bool>(m_device->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

  auto max_fps = GenApi::CFloatPtr(node_map->GetNode("AcquisitionFrameRate"))->GetMax();
  if (m_fps > max_fps) {
    Arena::SetNodeValue<bool>(node_map, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(node_map, "AcquisitionFrameRate", max_fps);
  } else {
    Arena::SetNodeValue<bool>(node_map, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(node_map, "AcquisitionFrameRate", static_cast<double>(m_fps));
  }

  std::size_t reached_binning_x;
  GenApi::CIntegerPtr pBinningHorizontal = m_device->GetNodeMap()->GetNode("BinningHorizontal");
  if (GenApi::IsWritable(pBinningHorizontal)) {
    size_t binning_x_to_set = m_horizontal_binning;
    if (binning_x_to_set < pBinningHorizontal->GetMin()) {
      binning_x_to_set = pBinningHorizontal->GetMin();
    } else if (binning_x_to_set > pBinningHorizontal->GetMax()) {
      binning_x_to_set = pBinningHorizontal->GetMax();
    }
    pBinningHorizontal->SetValue(binning_x_to_set);
    reached_binning_x = pBinningHorizontal->GetValue();
    std::cout << "Current binning horizontal : " << reached_binning_x << std::endl;
  } else {
    std::cout << "Binning horizantal value not readable." << std::endl;
  }

  std::size_t reached_binning_y;
  GenApi::CIntegerPtr pBinningVertical = m_device->GetNodeMap()->GetNode("BinningVertical");
  if (GenApi::IsWritable(pBinningVertical)) {
    size_t binning_y_to_set = m_vertical_binning;
    if (binning_y_to_set < pBinningVertical->GetMin()) {
      binning_y_to_set = pBinningVertical->GetMin();
    } else if (binning_y_to_set > pBinningVertical->GetMax()) {
      binning_y_to_set = pBinningVertical->GetMax();
    }
    pBinningHorizontal->SetValue(binning_y_to_set);
    reached_binning_y = pBinningVertical->GetValue();
    std::cout << "Current binning vertical : " << reached_binning_y << std::endl;
  } else {
    std::cout << "Binning vertical value not readable." << std::endl;
  }

  m_device->StartStream();
}

void ArenaCamera::stop_stream() { m_device->StopStream(); }

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

cv::Mat ArenaCamera::convert_to_image(Arena::IImage * pImage, const std::string & frame_id)
{
  cv::Mat image_cv =
    cv::Mat(pImage->GetHeight(), pImage->GetWidth(), CV_8UC1, (uint8_t *)pImage->GetData());

  cv::Mat image_bgr(image_cv.rows, image_cv.cols, CV_8UC3);
  cvtColor(image_cv, image_bgr, cv::COLOR_BayerBG2BGR);

  //  cv::resize(image_bgr, image_bgr, cv::Size(720, 465));
  if (m_resize_image) {
    //    cv::flip(image_bgr, image_bgr, -1);
    cv::resize(image_bgr, image_bgr, cv::Size(720, 465));
  }

  return image_bgr;
}

ArenaCamera::~ArenaCamera()
{
  std::cout << "Camera:" << m_cam_idx << " ~ArenaCamera()" << std::endl;
  m_continue_acquiring = false;
  stop_stream();
}
