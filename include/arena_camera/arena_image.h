#ifndef BUILD_ARENA_IMAGE_H
#define BUILD_ARENA_IMAGE_H

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

class Image{
public:
  Image(cv::Mat cv_image,
        uint64_t image_timestamp_ns,
        std::string encoding_format,
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

#endif // BUILD_ARENA_IMAGE_H
