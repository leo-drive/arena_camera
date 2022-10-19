#include "arena_camera/arena_camera_node.h"
#include "arena_camera/camera_settings.h"
#include "arena_camera/time_keeper_sequential.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <utility>
#include <chrono>

#include <signal.h>

ArenaCameraNode::ArenaCameraNode(const rclcpp::NodeOptions &node_options)
    : rclcpp::Node{"arena_camera_node", node_options} {
  auto camera_settings = read_camera_settings();
  m_arena_camera_handler = std::make_unique<ArenaCamerasHandler>();
  m_arena_camera_handler->create_cameras_from_settings(camera_settings);
  this->m_frame_id = camera_settings.get_frame_id();
  init_camera_info(camera_settings.get_camera_name(), camera_settings.get_url_camera_info());
  m_publisher = this->create_publisher<sensor_msgs::msg::Image>(
          create_camera_topic_name(camera_settings.get_camera_name()) + "/image" , rclcpp::SensorDataQoS());
  m_camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>(
          create_camera_topic_name(camera_settings.get_camera_name()) + "/camera_info" , rclcpp::SensorDataQoS());

  m_arena_camera_handler->set_image_callback(
      std::bind(&ArenaCameraNode::publish_image,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  m_arena_camera_handler->start_stream();
}

CameraSetting ArenaCameraNode::read_camera_settings() {

  const std::string cameras_param_name{"camera_names"};

  CameraSetting  camera_setting(
          declare_parameter("camera_name").template get<std::string>(),
        declare_parameter("frame_id").template get<std::string>(),
        declare_parameter("pixel_format").template get<std::string>(),
        declare_parameter("serial_no").template get<uint32_t>(),
        declare_parameter("fps").template get<uint32_t>(),
        declare_parameter("horizontal_binning").template get<uint32_t>(),
        declare_parameter("vertical_binning").template get<uint32_t>(),
        declare_parameter("resize_image").template get<bool>(),
        declare_parameter("camera_info_url").template get<std::string>());


  return camera_setting;
}


void ArenaCameraNode::publish_image(std::uint32_t camera_index,
                                    const cv::Mat& image) {
  const auto publisher_index = camera_index;

  sensor_msgs::msg::Image img_msg;
  std_msgs::msg::Header header;
  header.stamp = this->now();

  header.frame_id = m_frame_id;

  TimeKeeperSequental time_keeper("publish_image");
  time_keeper.AddTimePoint("start");

  try {
    //
      cv_bridge::CvImage img_bridge =
              cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
      (void) img_bridge;
//      img_bridge.toCompressedImageMsg(img_msg);
      img_bridge.toImageMsg(img_msg);

  } catch (...) {
      throw std::runtime_error("Runtime error, publish_image.");
  }

  time_keeper.AddTimePoint("img_bridge");

  m_publisher->publish(std::move(img_msg));

  if(m_camera_info_publisher){
      auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(m_camera_info->getCameraInfo());
      ci->header = img_msg.header;
      m_camera_info_publisher->publish(std::move(ci));
  }
  time_keeper.AddTimePoint("publish");

  // log with rclcpp
  RCLCPP_DEBUG(this->get_logger(), "publish_image: %s", time_keeper.StrLog().c_str());

}


void ArenaCameraNode::init_camera_info(std::string camera_name, std::string camera_info_url){
  m_camera_info  = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name);
  std::cout<<"camera_name :" << camera_name<<std::endl;
  std::cout<<"camera_info :" << camera_info_url<<std::endl;
  if (m_camera_info->validateURL(camera_info_url)) {
    m_camera_info->loadCameraInfo(camera_info_url);
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
  }

}


RCLCPP_COMPONENTS_REGISTER_NODE(ArenaCameraNode)