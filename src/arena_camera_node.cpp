#include "arena_camera/arena_camera_node.hpp"

#include "arena_camera/camera_settings.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rcutils/logging_macros.h>

#include <chrono>
#include <utility>

namespace arena_camera
{
ArenaCameraNode::ArenaCameraNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("arena_camera_node", node_options)
{
  auto camera_settings = get_camera_settings_from_params();
  m_arena_camera_handler = std::make_unique<ArenaCamerasHandler>();
  m_arena_camera_handler->create_camera_from_settings(camera_settings);
  this->m_frame_id = camera_settings.frame_id;
  this->m_use_ptp = camera_settings.use_ptp;

  const auto & camera_name = camera_settings.camera_name;

  init_camera_info(camera_name, camera_settings.url_camera_info);

  m_publisher = this->create_publisher<sensor_msgs::msg::Image>(
    create_camera_topic_name(camera_name) + "/image_raw", rclcpp::QoS{1}.reliable());
  m_rect_publisher = this->create_publisher<sensor_msgs::msg::Image>(
    create_camera_topic_name(camera_name) + "/image_rect_color", rclcpp::QoS{1}.reliable());
  m_compressed_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    create_camera_topic_name(camera_name) + "/image_raw/compressed", rclcpp::QoS{1}.reliable());
  m_camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    create_camera_topic_name(camera_name) + "/camera_info", rclcpp::QoS{1}.reliable());
  m_arena_camera_handler->set_image_callback(
    std::bind(&ArenaCameraNode::publish_image, this, std::placeholders::_1, std::placeholders::_2));
  m_arena_camera_handler->start_stream();

  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ArenaCameraNode::parameters_callback, this, std::placeholders::_1));
}

CameraSettings ArenaCameraNode::get_camera_settings_from_params()
{
  auto desc_fps = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::FloatingPointRange fps_range;
  fps_range.set__from_value(1.0).set__to_value(20.0).set__step(1.0);
  desc_fps.floating_point_range = {fps_range};

  auto desc_exposure = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::FloatingPointRange exposure_range;
  exposure_range.set__from_value(87.0).set__to_value(66000.0).set__step(1.0);
  desc_exposure.floating_point_range = {exposure_range};

  auto desc_gain = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::FloatingPointRange gain_range;
  gain_range.set__from_value(0.0).set__to_value(42.0).set__step(1.0);
  desc_gain.floating_point_range = {gain_range};

  auto desc_target_brightness = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::IntegerRange target_brightness_range;
  target_brightness_range.set__from_value(0).set__to_value(255).set__step(1);
  desc_target_brightness.integer_range = {target_brightness_range};

  auto desc_exposure_damping = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::FloatingPointRange exposure_damping_range;
  exposure_damping_range.set__from_value(0).set__to_value(100).set__step(1.0);
  desc_exposure_damping.floating_point_range = {exposure_damping_range};

  auto desc_balance_ratio = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::FloatingPointRange balance_ratio_range;
  balance_ratio_range.set__from_value(0).set__to_value(100).set__step(1.0);
  desc_balance_ratio.floating_point_range = {balance_ratio_range};
// describe balance ratio range for each red, green and blue channel
  auto desc_balance_ratio_red = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::FloatingPointRange balance_ratio_red_range;
  balance_ratio_red_range.set__from_value(0).set__to_value(100).set__step(1.0);
  desc_balance_ratio_red.floating_point_range = {balance_ratio_red_range};

  auto desc_balance_ratio_green = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::FloatingPointRange balance_ratio_green_range;
  balance_ratio_green_range.set__from_value(0).set__to_value(100).set__step(1.0);
  desc_balance_ratio_green.floating_point_range = {balance_ratio_green_range};

  auto desc_balance_ratio_blue = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::FloatingPointRange balance_ratio_blue_range;
  balance_ratio_blue_range.set__from_value(0).set__to_value(100).set__step(1.0);
  desc_balance_ratio_blue.floating_point_range = {balance_ratio_blue_range};

  auto camera_name = declare_parameter<std::string>("camera_name");
  auto frame_id = declare_parameter<std::string>("frame_id");
  auto pixel_format = declare_parameter<std::string>("pixel_format");
  auto serial_no = static_cast<uint32_t>(declare_parameter<int64_t>("serial_no"));
  auto fps = static_cast<uint32_t>(declare_parameter<int64_t>("fps", desc_fps));
  auto horizontal_binning = static_cast<uint32_t>(declare_parameter<int64_t>("horizontal_binning"));
  auto vertical_binning = static_cast<uint32_t>(declare_parameter<int64_t>("vertical_binning"));
  auto camera_info_url = declare_parameter<std::string>("camera_info_url");
  auto exposure_auto = declare_parameter<bool>("exposure_auto");
  auto exposure_value = declare_parameter<double>("exposure_value", desc_exposure);
  auto gain_auto = declare_parameter<bool>("gain_auto");
  auto gain_value = declare_parameter<double>("gain_value", desc_gain);
  auto enable_rectifying = declare_parameter<bool>("enable_rectifying");
  auto enable_compressing = declare_parameter<bool>("enable_compressing");
  auto use_default_device_settings = declare_parameter<bool>("use_default_device_settings");
  auto use_ptp = declare_parameter<bool>("use_ptp");
  auto target_brightness = declare_parameter<long>("target_brightness", desc_target_brightness);
  auto exposure_auto_limit_auto = declare_parameter<bool>("exposure_auto_limit_auto");
  auto exposure_auto_lower_limit = declare_parameter<double>("exposure_auto_lower_limit");
  auto exposure_auto_upper_limit = declare_parameter<double>("exposure_auto_upper_limit");
  auto exposure_damping = declare_parameter<double>("exposure_damping", desc_exposure_damping);
  auto lut_enable = declare_parameter<bool>("lut_enable");
  auto balance_white_auto = declare_parameter<bool>("balance_white_auto");
  auto balance_ratio_selector = static_cast<uint32_t>(declare_parameter<int64_t>("balance_ratio_selector"));

  //auto balance_ratio = declare_parameter<double>("balance_ratio", desc_balance_ratio);
  auto balance_ratio_red = declare_parameter<double>("balance_ratio.red", desc_balance_ratio_red);
  auto balance_ratio_green = declare_parameter<double>("balance_ratio.green", desc_balance_ratio_green);
  auto balance_ratio_blue = declare_parameter<double>("balance_ratio.blue", desc_balance_ratio_blue);

  CameraSettings camera_settings(
    camera_name, frame_id, pixel_format, serial_no, fps, horizontal_binning, vertical_binning,
    camera_info_url, exposure_auto, exposure_value, gain_auto, gain_value,
    enable_rectifying, enable_compressing, use_default_device_settings, use_ptp, target_brightness, exposure_auto_limit_auto, exposure_auto_lower_limit, exposure_auto_upper_limit, exposure_damping, lut_enable, balance_white_auto, balance_ratio_selector, balance_ratio_red, balance_ratio_green, balance_ratio_blue);

  return camera_settings;
}

void ArenaCameraNode::publish_image(std::uint32_t camera_index, std::shared_ptr<Image> image)
{
  sensor_msgs::msg::Image img_msg;
  std_msgs::msg::Header header;
  header.frame_id = m_frame_id;

  if (!m_use_ptp) {
    header.stamp = this->now();
  } else {
    if (image->ptp_status != "Slave") {
      rclcpp::Time image_stamp = rclcpp::Time(image->image_timestamp_ns);
      header.stamp = image_stamp;
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Check PTP Server. Status: " << image->ptp_status);
      // ROS zamanı ve image timestamp arasındaki farkı hesaplama
      rclcpp::Time ros_time = rclcpp::Clock().now();
      rclcpp::Duration difference = ros_time - image_stamp;
      auto nanoseconds_diff = difference.nanoseconds();

      // Farkı saniye ve milisaniye olarak ayırma
      auto seconds_diff = nanoseconds_diff / 1000000000;
      auto milliseconds_diff = (nanoseconds_diff % 1000000000) / 1000000;

      // İnsan tarafından okunabilir farkı ekrana yazdırma
      std::cout << "Difference: ";
      if (nanoseconds_diff >= 0) {
        std::cout << "+";
      } else {
        std::cout << "-";
      }
      std::cout << std::abs(seconds_diff) << " seconds, " << std::setw(3) <<
        milliseconds_diff << " milliseconds" << std::endl;
    } else {
      rclcpp::Time image_stamp = rclcpp::Time(image->image_timestamp_ns);
      header.stamp = image_stamp;
              // ROS zamanını elde etme
      rclcpp::Time ros_time = rclcpp::Clock().now();

              // ROS zamanı ve image timestamp arasındaki farkı hesaplama
              rclcpp::Duration difference = ros_time - image_stamp;
              auto nanoseconds_diff = difference.nanoseconds();

              // Farkı saniye ve milisaniye olarak ayırma
              auto seconds_diff = nanoseconds_diff / 1000000000;
              auto milliseconds_diff = (nanoseconds_diff % 1000000000) / 1000000;

              // İnsan tarafından okunabilir farkı ekrana yazdırma
              std::cout << "Difference: ";
              if (nanoseconds_diff >= 0) {
                  std::cout << "+";
              } else {
                  std::cout << "-";
              }
              std::cout << std::abs(seconds_diff) << " seconds, " << std::setw(3) <<
              milliseconds_diff << " milliseconds" << std::endl;

      // Zaman damgasını elde etme
      auto nanoseconds = image_stamp.nanoseconds();
      auto seconds = nanoseconds / 1000000000;
      auto milliseconds = (nanoseconds % 1000000000) / 1000000;

      // struct timespec nesnesi oluşturma
      struct timespec ts;
      ts.tv_sec = seconds;
      ts.tv_nsec = milliseconds * 1000000;

      // struct timespec'i tm struct'ına dönüştürme
      struct tm * timeinfo;
      time_t rawtime = ts.tv_sec;
      timeinfo = localtime(&rawtime);

      // İnsan tarafından okunabilir zamanı ekrana yazdırma
//      char buffer[80];
//      strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
//      std::cout << "Human-readable Time: " << buffer << "." << std::setw(3) << milliseconds <<
//      std::endl;
    }
  }

  try {
    cv_bridge::CvImage img_bridge =
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image->cv_image);
    (void)img_bridge;
    img_bridge.toImageMsg(img_msg);

  } catch (...) {
    throw std::runtime_error("Runtime error, publish_image.");
  }

  if (m_arena_camera_handler->get_enable_rectifying()) {
    sensor_msgs::msg::Image img_rect_msg;
    try {
      cv_bridge::CvImage img_bridge_rect =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8);
      (void)img_bridge_rect;

      cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(img_msg, img_msg.encoding);
      m_camera_model.fromCameraInfo(m_camera_info->getCameraInfo());
      m_camera_model.rectifyImage(cv_img_raw->image, img_bridge_rect.image);

      img_bridge_rect.toImageMsg(img_rect_msg);
    } catch (...) {
      throw std::runtime_error("Runtime error, publish_rectified_image.");
    }

    m_rect_publisher->publish(std::move(img_rect_msg));
  }

  if (m_arena_camera_handler->get_enable_compressing()) {
    sensor_msgs::msg::CompressedImage img_compressed_msg;
    try {
      cv_bridge::CvImage img_bridge_compressed =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image->cv_image);
      (void)img_bridge_compressed;

      img_bridge_compressed.toCompressedImageMsg(img_compressed_msg);
    } catch (...) {
      throw std::runtime_error("Runtime error, publish_compressed_image.");
    }

    m_compressed_publisher->publish(std::move(img_compressed_msg));
  }

  m_publisher->publish(std::move(img_msg));

  if (m_camera_info_publisher) {
    auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(m_camera_info->getCameraInfo());
    ci->header = img_msg.header;
    m_camera_info_publisher->publish(std::move(ci));
  }
}

void ArenaCameraNode::init_camera_info(std::string camera_name, std::string camera_info_url)
{
  m_camera_info = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name);
  if (m_camera_info->validateURL(camera_info_url)) {
    m_camera_info->loadCameraInfo(camera_info_url);
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
  }
}

rcl_interfaces::msg::SetParametersResult ArenaCameraNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  auto print_status = [this](const rclcpp::Parameter & parameter) {
    RCLCPP_INFO(this->get_logger(), "%s", parameter.get_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", parameter.value_to_string().c_str());
  };
  // create a vector to store the balance ratio values
  std::vector<double> balance_ratio_values;
  balance_ratio_values.resize(3);
  for (const auto & param : parameters) {
    if (param.get_name() == "fps") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        if (param.as_int() >= 1 && param.as_int() <= 20) {
          m_arena_camera_handler->set_fps(param.as_int());
          result.successful = true;
          print_status(param);
        }
      }
    }

    if (param.get_name() == "exposure_auto") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_auto_exposure(param.as_bool());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "enable_rectifying") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_enable_rectifying(param.as_bool());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "enable_compressing") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_enable_compressing(param.as_bool());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "exposure_value") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        m_arena_camera_handler->set_exposure_value(param.as_double());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "gain_auto") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_auto_gain(param.as_bool());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "gain_value") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        m_arena_camera_handler->set_gain_value(param.as_double());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "use_default_device_settings") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_use_default_device_settings(param.as_bool());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "target_brightness") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        m_arena_camera_handler->set_brightness(param.as_int());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "exposure_auto_limit_auto") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_exposure_auto_limit_auto(param.as_bool());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "exposure_auto_lower_limit") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        m_arena_camera_handler->set_exposure_auto_lower_limit(param.as_double());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "exposure_auto_upper_limit") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        m_arena_camera_handler->set_exposure_auto_upper_limit(param.as_double());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "exposure_damping") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        m_arena_camera_handler->set_exposure_damping(param.as_double());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "lut_enable") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_lut_enable(param.as_bool());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "balance_white_auto") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_balance_white_auto(param.as_bool());
        result.successful = true;
        std::cout << "BALANCE WHITE AUTO DEGISTI" << std::endl;
        print_status(param);
      }
    }

    if (param.get_name() == "balance_ratio_selector") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        m_arena_camera_handler->set_balance_ratio_selector(param.as_int());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "balance_ratio_selector"|| param.get_name() == "balance_ratio.red" || param.get_name() == "balance_ratio.green" || param.get_name() == "balance_ratio.blue") {

        if(param.get_type()== rclcpp::ParameterType::PARAMETER_DOUBLE || param.get_type()== rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          std::cout << "BALANCE DEGERI ELSE doube imis AFERIN" << std::endl;

         // set the red, green and blue values as 0=red, 1=green and 2=blue and push the balance_ratio_values vector separately
          if(param.get_name() == "balance_ratio.red")
          {
            balance_ratio_values[0] = param.as_double();
          }
          else if(param.get_name() == "balance_ratio.green")
          {
            balance_ratio_values[1] = param.as_double();
          }
          else if(param.get_name() == "balance_ratio.blue")
          {
            balance_ratio_values[2] = param.as_double();
          }
          // set the balance ratio values
          m_arena_camera_handler->set_balance_ratio(balance_ratio_values[0], balance_ratio_values[1], balance_ratio_values[2]);
          result.successful = true;

          print_status(param);
        }
     }
  }

  return result;
}

}  // namespace arena_camera

RCLCPP_COMPONENTS_REGISTER_NODE(arena_camera::ArenaCameraNode)