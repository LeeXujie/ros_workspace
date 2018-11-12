#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camnode");
  ros::NodeHandle nh_("~");
  if(!nh_.ok())return 0;

  int video_device, frame_rate;
  std::string camera_frame_id_, camera_name_, camera_info_url_;

  image_transport::ImageTransport it(nh_);
  image_transport::CameraPublisher imgPub = it.advertiseCamera("image_raw", 10);
  boost::shared_ptr<camera_info_manager::CameraInfoManager> caminfo_;

  nh_.param<int>("video_device", video_device, 0);
  nh_.param<int>("frame_rate", frame_rate, 30);
  nh_.param("camera_frame_id", camera_frame_id_, std::string("head_camera"));
  nh_.param("camera_name", camera_name_, std::string("head_camera"));
  nh_.param("camera_info_url", camera_info_url_, std::string(""));

  caminfo_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));

  // check for default camera info
  if (!caminfo_->isCalibrated())
  {
    caminfo_->setCameraName(camera_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = camera_frame_id_;
    camera_info.width = 640;
    camera_info.height = 480;
    caminfo_->setCameraInfo(camera_info);
  }

  cv::VideoCapture TheVideoCapturer;
  TheVideoCapturer.open(video_device);
  if (!TheVideoCapturer.isOpened()) {
    ROS_ERROR("Open camera device%d error!", video_device);
    return false;
  }
  ROS_INFO("Camera device%d openned, fps=%d", video_device, frame_rate);

  ros::Rate loop_rate(frame_rate);

  while (ros::ok())
  {
    if(TheVideoCapturer.grab())
    {
      cv::Mat image;
      TheVideoCapturer.retrieve(image);
      cv_bridge::CvImage out_msg;
      out_msg.header.stamp = ros::Time::now();
      out_msg.encoding = sensor_msgs::image_encodings::BGR8;
      out_msg.image = image;

      sensor_msgs::Image img_;
      out_msg.toImageMsg(img_);

      sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(caminfo_->getCameraInfo()));
      ci->header.frame_id = out_msg.header.frame_id;
      ci->header.stamp = out_msg.header.stamp;

      imgPub.publish(img_, *ci);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
