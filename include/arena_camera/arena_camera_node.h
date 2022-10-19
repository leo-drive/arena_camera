#ifndef BUILD_SRC_ARENA_CAMERA_SRC_ARENA_CAMERA_NODE_H_
#define BUILD_SRC_ARENA_CAMERA_SRC_ARENA_CAMERA_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <Arena/ArenaApi.h>

#include <chrono>
#include <thread>

#include "arena_camera/camera_settings.h"
#include "arena_cameras_handler.h"

class ArenaCameraNode: public ::rclcpp::Node {
public:
  explicit ArenaCameraNode(const rclcpp::NodeOptions & node_options);

  CameraSetting read_camera_settings();

private:
  class ProtectedPublisher;

  void publish_image(
    std::uint32_t camera_index,
    const cv::Mat& image);

  static ProtectedPublisher create_publishers(
    ::rclcpp::Node * node,
    CameraSetting camera_settings);

    void init_camera_info(std::string camera_name, std::string camera_info_url);

  static std::string create_camera_topic_name(std::string camera_name)
  {
    return "/lucid_vision/" + camera_name ;
  }

  std::unique_ptr < ArenaCamerasHandler > m_arena_camera_handler;

  using PublisherT = ::rclcpp::Publisher<::sensor_msgs::msg::Image>;
  using CameraInfoPublisherT = ::rclcpp::Publisher<::sensor_msgs::msg::CameraInfo>;

  PublisherT::SharedPtr m_publisher{};
  CameraInfoPublisherT::SharedPtr m_camera_info_publisher{};

  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info{};
  image_geometry::PinholeCameraModel m_camera_model;
  std::string m_frame_id;

};




#endif //BUILD_SRC_ARENA_CAMERA_SRC_ARENA_CAMERA_NODE_H_
