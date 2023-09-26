#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <memory>
#include <chrono>
#include <algorithm>
#include <sstream>

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

#include <Eigen/Dense>

#include "gflags/gflags.h"

DEFINE_int32(width_, 960, "model input width");
DEFINE_int32(height_, 384, "model input height");
DEFINE_string(image_topic_, "/apollo/sensor/camera/obstacle/front_6mm", "camera subscriber topic");

unsigned char CLIPVALUE(int val) {
  // Old method (if)
  val = val < 0 ? 0 : val;
  return val > 255 ? 255 : val;

  // New method (array)
  // return uchar_clipping_table[val + clipping_table_offset];
}

void YUV2RGB(const unsigned char y, const unsigned char u,
             const unsigned char v, unsigned char* r, unsigned char* g,
             unsigned char* b) {
  const int y2 = static_cast<int>(y);
  const int u2 = static_cast<int>(u) - 128;
  const int v2 = static_cast<int>(v) - 128;

  double r2 = y2 + (1.4065 * v2);
  double g2 = y2 - (0.3455 * u2) - (0.7169 * v2);
  double b2 = y2 + (2.041 * u2);

  *r = CLIPVALUE(r2);
  *g = CLIPVALUE(g2);
  *b = CLIPVALUE(b2);
}

void Yuyv2rgb(unsigned char* YUV, unsigned char* RGB, int NumPixels) {
  for (int i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6) {
    unsigned char y0 = (unsigned char)YUV[i + 0];
    unsigned char u = (unsigned char)YUV[i + 1];
    unsigned char y1 = (unsigned char)YUV[i + 2];
    unsigned char v = (unsigned char)YUV[i + 3];
    unsigned char r, g, b;
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
}

void yuyv2bgr(const unsigned char *yuv, unsigned char *bgr, const int len) {
  int i, j = 0;
  for(i = 0; i < len; i += 4) {
    const unsigned char *pyuv = yuv+i;
    unsigned char *pbgr = bgr+j;
    int r = (22987 * ((pyuv)[3] - 128)) >> 14;
    int g = (-5636 * ((pyuv)[1] - 128) - 11698 * ((pyuv)[3] - 128)) >> 14;
    int b = (29049 * ((pyuv)[1] - 128)) >> 14;
    (pbgr)[0] = (*(pyuv) + b);
    (pbgr)[1] = (*(pyuv) + g);
    (pbgr)[2] = (*(pyuv) + r);
    (pbgr)[3] = ((pyuv)[2] + b);
    (pbgr)[4] = ((pyuv)[2] + g);
    (pbgr)[5] = ((pyuv)[2] + r);
    j+= 6;
  }
}

#define XPERF_CODE_BLOCK_BEGIN() std::chrono::system_clock::time_point _tmBegin, _tmEnd; \
                                 _tmBegin = std::chrono::system_clock::now();

#define XPERF_CODE_BLOCK_END(fnName) _tmEnd = std::chrono::system_clock::now(); \
                                 std::cerr << "[" << fnName << "]" << std::chrono::duration_cast<std::chrono::milliseconds>(_tmEnd - _tmBegin).count() << " ms\n"; \
                                 _tmBegin = _tmEnd;

static int nDeviceNum = 0;
//-----------------------------------------------------------------------------------------------------------------------------------------

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter() : it_(nh_) {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(FLAGS_image_topic_, 10, &ImageConverter::imageCb, this);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& input) {
     cv::Mat cv_im;
     try {
       cv_im = cv::Mat(input->height, input->width, CV_8UC3);
       int pixel_num = input->width * input->height;

       if (input->encoding.compare("yuyv") == 0) {
         unsigned char *yuv = (unsigned char *)&(input->data[0]);
         std::cout << "YUYV\n";
         Yuyv2rgb((uint8_t *)yuv, (uint8_t *)cv_im.data, pixel_num);
         cv::cvtColor(cv_im, cv_im, CV_RGB2BGR);         
       } else {
         cv_im = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8)->image;
       }
     } catch (cv_bridge::Exception& e) {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return ;
     }

     XPERF_CODE_BLOCK_BEGIN();
     double ratio = FLAGS_width_ * 1. / cv_im.cols;
     int offset_y = (cv_im.rows - static_cast<int>(cv_im.rows * ratio)) / 2;
     cv::Mat img(FLAGS_width_, FLAGS_height_, CV_8UC3);
     cv::Rect roi(0, offset_y, cv_im.cols, static_cast<int>(cv_im.rows * ratio));
     cv::resize(cv_im(roi), img, cv::Size(FLAGS_width_, FLAGS_height_), 0, 0);
     XPERF_CODE_BLOCK_END("Image_Preprocess");

     cv::imshow("IMAGE-A", cv_im);
     cv::imshow("IMAGE-B", img);
     cv::waitKey(10);
  }
};

//=========================================================================================================================================


int main(int argc, char **argv)
{
#if (CV_MAJOR_VERSION == 2)
  using cv::gpu::getCudaEnabledDeviceCount;
  using cv::gpu::GpuMat;
#elif (CV_MAJOR_VERSION == 3)
  using cv::cuda::getCudaEnabledDeviceCount
  using cv::cuda::GpuMat;
#endif
  nDeviceNum = getCudaEnabledDeviceCount();

  if (nDeviceNum < 1) {
    std::cout << "[" << nDeviceNum << "] OpenCV Compile without CUDA\n";
  } else {
    cv::Mat cmat = cv::Mat::zeros(cv::Size(1920, 1080), CV_16UC1);
    GpuMat gmat(cmat);
    std::cout << "OpenCV Compile with CUDA\n";
  }

  ros::init(argc, argv, "CameraViewer");

  cv::namedWindow("IMAGE-A"/*, CV_WINDOW_NORMAL*/);
  cv::namedWindow("IMAGE-B"/*, CV_WINDOW_NORMAL*/);

  ImageConverter ic;

  ros::spin();

  return 0;
}
