#ifndef ROS_VIDEO_CAPTURE_H_
#define ROS_VIDEO_CAPTURE_H_

#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/String.h"
#include "velodyne_msgs/VelodyneScan.h"

#include "connection_settings.h"
#include "rtc/scalable_track_source.h"
#include "api/peer_connection_interface.h"

class ROSVideoCapture : public ScalableVideoTrackSource
{
public:
  explicit ROSVideoCapture(ConnectionSettings cs);
  ~ROSVideoCapture();

  void Destroy();

  // ROS Callback
  void ROSCallbackRaw(const sensor_msgs::ImageConstPtr &image);
  void ROSCallbackCompressed(const sensor_msgs::CompressedImageConstPtr &image);
  void CaptureData(rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel);
  void CaptureStart();
  void ROSDataCallback(const std_msgs::String::ConstPtr& msg);
  void ROSVelodyneCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg);
  // ros::NodeHandle *nh;
  rtc::scoped_refptr<webrtc::DataChannelInterface> dc;

private:
  ros::NodeHandle nh;
  static uint32_t ConvertEncodingType(const std::string encoding);
  void ROSCallback(ros::Time ros_time, const uint8_t* sample, size_t sample_size, int src_width, int src_height, uint32_t fourcc);

  ros::AsyncSpinner* spinner_;
  ros::Subscriber sub_;
  ros::Subscriber sub_data;
  ros::Subscriber sub_velodyne;
};

#endif