/****************By lxj****************/

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/cameraparameters.h>
#include <aruco/dictionary.h>
#include <memory>
#include <opencv2/imgproc/imgproc.hpp>
#include "marker_mapper/markermapper.h"
#include "debug.h"
#include "sglviewer.h"
#include "aruco/markerlabeler.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

using namespace cv;
using namespace std;
using namespace aruco;

int waitTime=5;
static char cvKey;
static int frame=0;
int frameIncrement;
cv::Mat image1,image2;
std::shared_ptr<aruco_mm::MarkerMapper> AMM;

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  if(cvKey!=27) {
    image1 = cv_bridge::toCvShare(msg, "bgr8")->image;
    image1.copyTo(image2);

    frame++;
    if (frame % frameIncrement != 0)return;
    AMM->process(image1, frame);
    AMM->drawDetectedMarkers(image2);

    cv::imshow("image", image2);
    cvKey = cv::waitKey(waitTime);
    if (cvKey == 's') waitTime = waitTime == 0 ? 5 : 0;
    if (cvKey == 'w') cv::imwrite("image.jpg", image2);

    cout << "frame " << frame << endl;
  } 
}

int main(int argc, char **argv) {
  bool saveFrameToPCD;
  float markerSize;
  int ref_Marker_Id;
  string camerafile, outBaseName, dictionary, savePath;

  ros::init(argc, argv, "marker_mapper");
  ros::NodeHandle nh("~");
  if(!nh.ok())return 0;

  nh.param<std::string>("camerafile", camerafile, "camera_name.yaml");
  nh.param<std::string>("dictionary", dictionary, "ALL_DICTS");
  nh.param<std::string>("outBaseName", outBaseName, "marker_map");
  nh.param<float>("marker_size", markerSize, 0.3);
  nh.param<int>("reference_marker_id", ref_Marker_Id, 0);
  nh.param<int>("frameIncrement", frameIncrement, 1);
  nh.param<std::string>("savePath", savePath, "~/");
  nh.param<bool>("saveFrameToPCD", saveFrameToPCD, false);

  aruco::CameraParameters Camera;
  Camera.readFromXMLFile(camerafile);
  
  AMM = aruco_mm::MarkerMapper::create();
  AMM->setParams(Camera, markerSize, ref_Marker_Id);
  AMM->getMarkerDetector().setDictionary(dictionary);
  cout<<AMM->getMarkerDetector().getMarkerLabeler()->getName()<<" "<<dictionary<<endl;

  cerr << "Press esc to end video processing" << endl;
  cerr << "Press 's' to start/stop video" << endl;
  cerr << "Press 'w' to take a snapshot" << endl;

  ros::Subscriber sub_image = nh.subscribe("image_raw", 10, imageCallback);

  ros::Rate loop_rate(30);

  while (ros::ok()) {
    if (cvKey == 27) {
      cv::destroyAllWindows();
      cout << "Finish processing video." << endl;
      cout << "Starting optimization." << endl;
      AMM->optimize();

      AMM->saveToPcd(savePath + outBaseName + ".pcd", saveFrameToPCD);
      AMM->saveFrameSetPosesToFile(savePath + outBaseName + ".log");
      AMM->getMarkerMap().saveToFile(savePath + outBaseName + ".yml");
      AMM->getCameraParams().saveToFile(savePath + outBaseName + "-cam.yml");

      cout << "\e[1;32mOK! Files have been saved to " << savePath << "\e[0m" << endl;

      ros::shutdown();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}