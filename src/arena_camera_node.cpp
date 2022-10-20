#include "arena_camera/arena_camera_node.h"

#include "arena_camera/camera_settings.h"

#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rcutils/logging_macros.h>

#include <chrono>
#include <utility>

ArenaCameraNode::ArenaCameraNode(rclcpp::NodeOptions node_options)
: rclcpp::Node("arena_camera_node", node_options.allow_undeclared_parameters(true))
{
  auto camera_settings = read_camera_settings();
  m_arena_camera_handler = std::make_unique<ArenaCamerasHandler>();
  m_arena_camera_handler->create_cameras_from_settings(camera_settings);
  this->m_frame_id = camera_settings.get_frame_id();
  init_camera_info(camera_settings.get_camera_name(), camera_settings.get_url_camera_info());
  m_publisher = this->create_publisher<sensor_msgs::msg::Image>(
    create_camera_topic_name(camera_settings.get_camera_name()) + "/image",
    rclcpp::SensorDataQoS());
  m_camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    create_camera_topic_name(camera_settings.get_camera_name()) + "/camera_info",
    rclcpp::SensorDataQoS());

  m_arena_camera_handler->set_image_callback(
    std::bind(&ArenaCameraNode::publish_image, this, std::placeholders::_1, std::placeholders::_2));
  m_arena_camera_handler->start_stream();

  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ArenaCameraNode::parameters_callback, this, std::placeholders::_1));
}

CameraSetting ArenaCameraNode::read_camera_settings()
{
  m_auto_exposure_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  m_auto_exposure_descriptor.name ="exposure_target";
  m_auto_exposure_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

  rcl_interfaces::msg::FloatingPointRange exposure_range;
  exposure_range.set__from_value(1.0).set__to_value(20.0).set__step(1.0);
  m_auto_exposure_descriptor.floating_point_range = {exposure_range};

  m_fps_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::IntegerRange fps_range;
  fps_range.set__from_value(1).set__to_value(20).set__step(1);
  m_fps_descriptor.integer_range = {fps_range};

  std::cout << "read_camera_settings" << std::endl;
  declare_parameter<double_t>("exposure_target", 90.0, m_auto_exposure_descriptor);
  std::cout << "read_camera_settings" << std::endl;

  CameraSetting camera_setting(
    declare_parameter<std::string>("camera_name"), declare_parameter<std::string>("frame_id"),
    declare_parameter<std::string>("pixel_format"),
    static_cast<uint32_t>(declare_parameter<int64_t>("serial_no")),
    static_cast<uint32_t>(declare_parameter<int64_t>("fps", m_fps_descriptor)),
    static_cast<uint32_t>(declare_parameter<int64_t>("horizontal_binning")),
    static_cast<uint32_t>(declare_parameter<int64_t>("vertical_binning")),
    declare_parameter<bool>("resize_image"), declare_parameter<std::string>("camera_info_url"),
    declare_parameter<bool>("exposure_auto"),
    static_cast<float>(declare_parameter<double>("exposure_target", m_auto_exposure_descriptor)),
    declare_parameter<bool>("gain_auto"), declare_parameter<float>("gain_target"),
    declare_parameter<float>("gamma_target"));

  return camera_setting;
}

void ArenaCameraNode::publish_image(std::uint32_t camera_index, const cv::Mat & image)
{
  sensor_msgs::msg::Image img_msg;
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = m_frame_id;

  try {
    cv_bridge::CvImage img_bridge =
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
    (void)img_bridge;
    img_bridge.toImageMsg(img_msg);

  } catch (...) {
    throw std::runtime_error("Runtime error, publish_image.");
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
  std::cout << "camera_name :" << camera_name << std::endl;
  std::cout << "camera_info :" << camera_info_url << std::endl;
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

  for (const auto & param : parameters) {
    if (param.get_name() == "fps") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        if (param.as_int() >= 1 && param.as_int() <= 20) {
          m_arena_camera_handler->set_fps(param.as_int());
          result.successful = true;
        }
      }
    }

    if (param.get_name() == "exposure_auto") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_auto_exposure(param.as_bool());
        result.successful = true;
      }
    }

    if (param.get_name() == "exposure_target") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        m_arena_camera_handler->set_exposure_value(param.as_double());
        result.successful = true;
      }
    }

    if (param.get_name() == "gain_auto") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_auto_gain(param.as_bool());
        result.successful = true;
      }
    }

    if (param.get_name() == "gain_target") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        m_arena_camera_handler->set_gain_value(param.as_double());
        result.successful = true;
      }
    }

    if (param.get_name() == "gamma_target") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        m_arena_camera_handler->set_gamma_value(param.as_double());
        result.successful = true;
      }
    }

    RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());
  }

  return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ArenaCameraNode)