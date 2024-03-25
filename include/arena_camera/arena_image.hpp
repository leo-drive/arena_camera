#ifndef BUILD_ARENA_IMAGE_HPP
#define BUILD_ARENA_IMAGE_HPP

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

namespace arena_camera
{
class Image
{
public:
  Image(
    cv::Mat cv_image, uint64_t image_timestamp_ns, std::string encoding_format,
    std::string ptp_status)
  {
    this->cv_image = cv_image;
    this->image_timestamp_ns = image_timestamp_ns;
    this->encoding_format = encoding_format;
    this->ptp_status = ptp_status;
  }

  cv::Mat cv_image;
  uint64_t image_timestamp_ns;
  std::string encoding_format;
  std::string ptp_status;
};
}  // namespace arena_camera

#endif  // BUILD_ARENA_IMAGE_HPP
