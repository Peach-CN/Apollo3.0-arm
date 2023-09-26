sensor_msgs::Image VehicleNode::compressImg(const sensor_msgs::Image &img,int width, int height){

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC3);
  cv::Mat cvImg = cv_ptr->image;
  cv::Mat cvCompImg;
  cv::resize(cvImg,cvCompImg,cv::Size(width,height));
  sensor_msgs::ImagePtr compImg = cv_bridge::CvImage(std_msgs::Header(),"rgb8",cvCompImg).toImageMsg();
  return (*compImg);
}

