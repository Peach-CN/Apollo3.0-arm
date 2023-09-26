#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#if (CV_MAJOR_VERSION == 2)
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/core/gpumat.hpp>
#elif (CV_MAJOR_VERSION == 3)
#include <opencv2/core.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#endif

#include <memory>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <sstream>

#include <Eigen/Dense>

#include "gflags/gflags.h"
#include "boost/filesystem.hpp"

#include "modules/common/util/file.h"

using apollo::common::util::PathExists;

DEFINE_string(image_topic_, "/apollo/sensor/camera/obstacle/front_6mm", "camera publisher topic");
DEFINE_string(image_path_, "/apollo/image/", "Path to image data.");

#define XPERF_CODE_BLOCK_BEGIN() std::chrono::system_clock::time_point _tmBegin, _tmEnd; \
                                 _tmBegin = std::chrono::system_clock::now();

#define XPERF_CODE_BLOCK_END(fnName) _tmEnd = std::chrono::system_clock::now(); \
                                 std::cerr << "[" << fnName << "]" << std::chrono::duration_cast<std::chrono::milliseconds>(_tmEnd - _tmBegin).count() << " ms\n"; \
                                 _tmBegin = _tmEnd;

//==========================================================================================================================================

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::vector<std::string> v_images;
  namespace fs = boost::filesystem;
  if (!fs::exists(FLAGS_image_path_) || !fs::is_directory(FLAGS_image_path_)) {
    return -1;
  }

  fs::directory_iterator it(FLAGS_image_path_);
  fs::directory_iterator endit;

  while (it != endit) {
    if (fs::is_regular_file(*it)) {
      std::string temp_path = it->path().filename().string();
      v_images.push_back(FLAGS_image_path_ + temp_path);
    }
    ++it;
  }
 
  ros::init(argc, argv, "CameraPublisher");

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Publisher image_pub_;

  image_pub_ = it_.advertise(FLAGS_image_topic_, 1);

  int index = 0;
  ros::Rate loop_rate(5);
  while (nh_.ok() && v_images.size() > 0) {
    cv::Mat img;

    if (index >= v_images.size()) index = 0;
    std::cout << "reading " << v_images[index] << "\n";
    img = cv::imread(v_images[index], CV_LOAD_IMAGE_COLOR);
    ++index;
    cv::waitKey(10);
    std::cout << "[" << img.cols << ", " << img.rows << "]\n";
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

    image_pub_.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  ros::shutdown();

  return 0;
}
