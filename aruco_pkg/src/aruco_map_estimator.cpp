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

MarkerDetector theMarkerDetector;
Mat theInputImage, theCopyImage;
std::vector<Marker> detected_markers;
MarkerMap theMarkerMapConfig;
MarkerMapPoseTracker theMSPoseTracker;
CameraParameters theCameraParameters;
float theMarkerSize = -1;
int ref_id = 0;

char cam_pose[100];
char cam_vect[100];

ros::Time current_time;
ros::Publisher odom_pub;

double qx, qy, qz, qw, tx, ty, tz;

image_transport::Publisher image_pub;

void getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in_, double &qx_,
                                             double &qy_, double &qz_, double &qw_,
                                             double &tx_, double &ty_, double &tz_);

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    theInputImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    theInputImage.copyTo(theCopyImage);
    detected_markers = theMarkerDetector.detect(theInputImage, theCameraParameters, theMarkerSize);

    current_time = ros::Time::now();

    if (theMSPoseTracker.isValid())
      if (theMSPoseTracker.estimatePose(detected_markers)) {
        Mat RTMatrix = theMSPoseTracker.getRTMatrix();
        getQuaternionAndTranslationfromMatrix44(RTMatrix, qx, qy, qz, qw, tx, ty, tz);
        Mat vect = (Mat_<float>(3, 1) << 0.0, 0.0, 1.0);
        Mat camPosMatrix, camVecMatrix;
        Mat RTInv=RTMatrix.inv();
        camPosMatrix=RTInv(Rect(3,0,1,3)).clone();
        camVecMatrix=RTInv(Range(0,3),Range(0,3))*vect;

//        Mat rMatrix, tMatrix;
//        Rodrigues(theMSPoseTracker.getRvec(), rMatrix);
//        tMatrix = theMSPoseTracker.getTvec();
//        rMatrix.convertTo(rMatrix,CV_32F);
//        tMatrix.convertTo(tMatrix,CV_32F);
//        camPosMatrix = rMatrix.inv() * (-tMatrix);
//        camVecMatrix = rMatrix.inv() * vect;

        sprintf(cam_pose, "Camera Position: px = %f, py = %f, pz = %f ",
                camPosMatrix.at<float>(0, 0), camPosMatrix.at<float>(1, 0), camPosMatrix.at<float>(2, 0));
        sprintf(cam_vect, "Camera Direction: dx = %f, dy = %f, dz = %f",
                camVecMatrix.at<float>(0, 0), camVecMatrix.at<float>(1, 0), camVecMatrix.at<float>(2, 0));

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = camPosMatrix.at<float>(0, 0);
        odom.pose.pose.position.y = camPosMatrix.at<float>(1, 0);
        odom.pose.pose.position.z = abs(camPosMatrix.at<float>(2, 0));
        odom.pose.pose.orientation.x = qx;
        odom.pose.pose.orientation.y = qy;
        odom.pose.pose.orientation.z = qz;
        odom.pose.pose.orientation.w = qw;
        odom_pub.publish(odom);
      }

    for (auto idx : theMarkerMapConfig.getIndices(detected_markers)) {
      detected_markers[idx].draw(theCopyImage, Scalar(0, 0, 255), 2, true);
      CvDrawingUtils::draw3dAxis(theCopyImage, detected_markers[idx], theCameraParameters);
      // CvDrawingUtils::draw3dCube(TheInputImageCopy,detected_markers[idx],TheCameraParameters);
    }

    putText(theCopyImage, cam_pose, Point(10, 13), CV_FONT_NORMAL, 0.5, Scalar(0, 255, 255));
    putText(theCopyImage, cam_vect, Point(10, 30), CV_FONT_NORMAL, 0.5, Scalar(0, 255, 255));

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = current_time;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = theCopyImage;
    image_pub.publish(out_msg.toImageMsg());

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {
  string camerafile, mapfile;

  ros::init(argc, argv, "aruco_map_estimator");
  ros::NodeHandle nh("~");
  if(!nh.ok())return 0;

  nh.param<std::string>("camerafile", camerafile, "camera_name.yaml");
  nh.param<std::string>("mapfile", mapfile, "map.yml");
  nh.param<float>("marker_size", theMarkerSize, 0.3);
  nh.param<int>("reference_marker_id", ref_id, 0);


  image_transport::ImageTransport it(nh);
  image_pub = it.advertise("cam_image", 10);
  ros::Subscriber sub_image = nh.subscribe("image_raw", 10, imageCallback);
  odom_pub = nh.advertise<nav_msgs::Odometry>("vo_data", 10);

  theMarkerDetector.getParameters().detectEnclosedMarkers(true);
  theCameraParameters.readFromXMLFile(camerafile);
  theMarkerMapConfig.readFromFile(mapfile);
  theMarkerDetector.setDictionary(theMarkerMapConfig.getDictionary());
  if (theMarkerMapConfig.isExpressedInPixels() && theMarkerSize > 0)
    theMarkerMapConfig = theMarkerMapConfig.convertToMeters(theMarkerSize);

  if (theCameraParameters.isValid() && theMarkerMapConfig.isExpressedInMeters()) {
    theMSPoseTracker.setParams(theCameraParameters, theMarkerMapConfig);
    theMarkerSize = cv::norm(theMarkerMapConfig[0][0] - theMarkerMapConfig[0][1]);
  }

  ros::Rate loop_rate(30);

  while (ros::ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


void getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in_, double &qx_,
                                             double &qy_, double &qz_, double &qw_,
                                             double &tx_, double &ty_, double &tz_)
{
  // get the 3d part of matrix and get quaternion
  assert(M_in_.total() == 16);
  cv::Mat M;
  M_in_.convertTo(M, CV_64F);
  cv::Mat r33 = cv::Mat(M, cv::Rect(0, 0, 3, 3));
  // use now eigen
  Eigen::Matrix3d e_r33;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      e_r33(i, j) = M.at<double>(i, j);

  // now, move to a angle axis
  Eigen::Quaterniond q(e_r33);
  qx_ = q.x();
  qy_ = q.y();
  qz_ = q.z();
  qw_ = q.w();

  tx_ = M.at<double>(0, 3);
  ty_ = M.at<double>(1, 3);
  tz_ = M.at<double>(2, 3);
}
