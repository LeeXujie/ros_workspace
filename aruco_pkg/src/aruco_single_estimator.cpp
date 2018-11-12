/****************By lxj****************/

#include <Eigen/Geometry>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/time.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

using namespace cv;
using namespace std;
using namespace aruco;

MarkerDetector mDetector;
Mat theInputImage, theCopyImage;
std::vector<Marker> theMarkers;
std::map<uint32_t, MarkerPoseTracker> mTracker;
CameraParameters theCameraParameters;
float theMarkerSize = -1;
int ref_id = 0;

char cam_pose[100];
char cam_vect[100];

image_transport::Publisher image_pub;

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    theInputImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    theInputImage.copyTo(theCopyImage);
    theMarkers = mDetector.detect(theInputImage);
    for (auto &marker : theMarkers)
      if(mTracker[marker.id].estimatePose(marker, theCameraParameters, theMarkerSize))
      {
        if (ref_id == marker.id) {

          Mat RTMatrix=mTracker[ref_id].getRTMatrix();
          Mat vect = (Mat_<float>(3, 1) << 0.0, 0.0, 1.0);
          Mat camPosMatrix, camVecMatrix;
          Mat RTInv=RTMatrix.inv();
          camPosMatrix=RTInv(Rect(3,0,1,3)).clone();
          camVecMatrix=RTInv(Range(0,3),Range(0,3))*vect;

//          Mat rMatrix, tMatrix;
//          Rodrigues(marker.Rvec, rMatrix);
//          tMatrix = (Mat_<float>(3, 1) << marker.Tvec.at<float>(0,0), marker.Tvec.at<float>(1,0), marker.Tvec.at<float>(2,0));
//          camPosMatrix = rMatrix.inv() * (-tMatrix);
//          camVecMatrix = rMatrix.inv() * vect;

          sprintf(cam_pose, "Camera Position: px = %f, py = %f, pz = %f ",
                  camPosMatrix.at<float>(0,0), camPosMatrix.at<float>(1, 0), camPosMatrix.at<float>(2, 0));
          sprintf(cam_vect, "Camera Direction: dx = %f, dy = %f, dz = %f",
                  camVecMatrix.at<float>(0, 0), camVecMatrix.at<float>(1, 0), camVecMatrix.at<float>(2, 0));

          CvDrawingUtils::draw3dAxis(theCopyImage, marker, theCameraParameters);
        }
        marker.draw(theCopyImage, Scalar(0, 0, 255), 2, true);
      }

    putText(theCopyImage, cam_pose, Point(10, 13), CV_FONT_NORMAL, 0.5, Scalar(0, 255, 255));
    putText(theCopyImage, cam_vect, Point(10, 30), CV_FONT_NORMAL, 0.5, Scalar(0, 255, 255));

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = theCopyImage;
    image_pub.publish(out_msg.toImageMsg());

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {
  string dictionary, camerafile;

  ros::init(argc, argv, "aruco_single_estimator");
  ros::NodeHandle nh("~");
  if(!nh.ok())return 0;

  nh.param<std::string>("camerafile", camerafile, "camera_name.yaml");
  nh.param<std::string>("dictionary", dictionary, "ALL_DICTS");
  nh.param<float>("marker_size", theMarkerSize, 0.3);
  nh.param<int>("reference_marker_id", ref_id, 0);


  image_transport::ImageTransport it(nh);
  image_pub = it.advertise("cam_image", 10);
  ros::Subscriber sub_image = nh.subscribe("image_raw", 10, imageCallback);

  mDetector.setDictionary(dictionary);
  mDetector.getParameters().detectEnclosedMarkers(true);
  theCameraParameters.readFromXMLFile(camerafile);

  ros::Rate loop_rate(30);

  while (ros::ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
