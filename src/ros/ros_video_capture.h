#ifndef ROS_VIDEO_CAPTURE_H_
#define ROS_VIDEO_CAPTURE_H_

#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/String.h"

#include "connection_settings.h"
#include "rtc/scalable_track_source.h"

class ROSVideoCapture : public ScalableVideoTrackSource
{
public:
  explicit ROSVideoCapture(ConnectionSettings cs);
  ~ROSVideoCapture();

  void Destroy();

  // ROS Callback
  void ROSCallbackRaw(const sensor_msgs::ImageConstPtr &image);
  void ROSCallbackCompressed(const sensor_msgs::CompressedImageConstPtr &image);
  // virtual void ROSDataCallback(const std_msgs::String::ConstPtr& msg);
  ros::NodeHandle *nh;

private:
  static uint32_t ConvertEncodingType(const std::string encoding);
  void ROSCallback(ros::Time ros_time, const uint8_t* sample, size_t sample_size, int src_width, int src_height, uint32_t fourcc);

  ros::AsyncSpinner* spinner_;
  ros::Subscriber sub_;
  ros::Subscriber sub_data;
};

#endif