//加载示教路径点文件，将其发布出去显示到RViz
//订阅fusion_filter滤波后的里程计消息，在示教路径点中查找与当前相机位姿对应的点

#include "ReadFile.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <aruco_pkg/robot_stop.h>

class pathTracker {
private:
  ros::NodeHandle nh;
  nav_msgs::Path path;
  ros::ServiceServer srv;
  ros::Subscriber sub;
  ros::Publisher path_pub;
  ros::Publisher teachPoint_pub;
  ros::Publisher targetPoint_pub;
  ros::Publisher twist_pub;
  ros::Publisher robotPose_pub;
  std::string path_file;
  geometry_msgs::Twist moto_control;
  geometry_msgs::PoseStamped this_pose_stamped;

  Mutex m_lock;
  Mat teachPath;
  float angle;
  bool forward_flag,stop_flag;
  int n, foresee, targetPosition;
  int rowcount, colcount, speedcount;
  int initialCount, currentPositon, lastPosition;

  float calculateDistance(Point3f pt1, Point3f pt2) {
    return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) +
                (pt1.y - pt2.y) * (pt1.y - pt2.y) +
                (pt1.z - pt2.z) * (pt1.z - pt2.z));
  }

  float getAngelOfTwoVector(Point2f &pt1, Point2f &pt2, Point2f &c) {
    float theta = atan2(pt1.x - c.x, pt1.y - c.y) - atan2(pt2.x - c.x, pt2.y - c.y);
    if (theta > CV_PI) theta -= 2 * CV_PI;
    if (theta < -CV_PI) theta += 2 * CV_PI;
    theta = theta * 180.0 / CV_PI;
    return theta;
  }

public:
  pathTracker() : nh("~"),foresee(25) {
    nh.param<std::string>("path_file", path_file, "navigation_path.txt");

    path.header.frame_id = "odom";
    path.header.stamp = ros::Time(0);
    path_pub = nh.advertise<nav_msgs::Path>("teaching_path", 1, true);
    teachPoint_pub = nh.advertise<visualization_msgs::Marker>("teaching_point", 1);
    targetPoint_pub = nh.advertise<visualization_msgs::Marker>("target_point", 1);
    twist_pub = nh.advertise<geometry_msgs::Twist>("/turtlebot_teleop/cmd_vel",1);
    robotPose_pub = nh.advertise<geometry_msgs::PoseStamped>("robotpose",1);
    sub = nh.subscribe("/vo_data", 10, &pathTracker::pose_track, this);
    srv = nh.advertiseService("stop_robot",&pathTracker::stop,this);

    readFile_();
    initialCount = 0;
    speedcount = 0;
    targetPosition = rowcount-500;
    forward_flag=true;
    stop_flag=false;
  }
  ~pathTracker()
  {
  }

  bool stop(aruco_pkg::robot_stop::Request &req, aruco_pkg::robot_stop::Response &res)
  {
    m_lock.lock();
    if(req.stop_flag){
      stop_flag=true;
      geometry_msgs::Twist robotStop;
      robotStop.linear.x=0;
      robotStop.angular.z=0;
      twist_pub.publish(robotStop);
      res.robot_status="robot is stopped!\n";}
    if(!req.stop_flag){stop_flag=false;res.robot_status="robot is running!\n";}
    m_lock.unlock();
  }

  void robotPosePub(cv::Point3f robotPosition_,Eigen::Quaterniond robotOrientation_)
  {
    geometry_msgs::PoseStamped robotPose;
    robotPose.header.frame_id="odom";
    robotPose.header.stamp=ros::Time::now();
    robotPose.pose.position.x=robotPosition_.x;
    robotPose.pose.position.y=robotPosition_.y;
    robotPose.pose.position.z=robotPosition_.z;
    robotPose.pose.orientation.w=robotOrientation_.w();
    robotPose.pose.orientation.x=robotOrientation_.x();
    robotPose.pose.orientation.y=robotOrientation_.y();
    robotPose.pose.orientation.z=robotOrientation_.z();
    robotPose_pub.publish(robotPose);
  }

  void navigation(cv::Point3f camPosition_,Eigen::Quaterniond camPose_,cv::Point3f teachPoint_)
  {
    Eigen::AngleAxisd t_V(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0));
    Eigen::Matrix3d robotPose = camPose_.matrix()*t_V.matrix();

    Mat robotPoseMat;
    cv::eigen2cv(robotPose,robotPoseMat);
    Mat vect = (Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
    Mat robotVect = robotPoseMat * vect;
    robotVect.convertTo(robotVect,CV_32F);

    Point2f c(0.0, 0.0);
    Point2f pt1(teachPoint_.x-camPosition_.x, teachPoint_.z-camPosition_.z);
    Point2f pt2(robotVect.at<float>(0,0), robotVect.at<float>(2,0));
    if(!forward_flag){pt2.x=-robotVect.at<float>(0,0);pt2.y=-robotVect.at<float>(2,0);}
    angle=getAngelOfTwoVector(pt1,pt2,c);
    
    if(speedcount<100)speedcount++;
    if(forward_flag){
      robotPosePub(camPosition_,camPose_);
      if(abs(n)<foresee)moto_control.linear.x=0.006*n+0.05;
      else moto_control.linear.x=0.01*speedcount*0.2;
    }
    else{
      Eigen::AngleAxisd V(M_PI, Eigen::Vector3d(0.0, 1.0, 0.0));
      Eigen::Quaterniond VQ(V);
      robotPosePub(camPosition_,camPose_*VQ);
      if(abs(n)<foresee)moto_control.linear.x=0.006*n-0.05;
      else moto_control.linear.x=-0.01*speedcount*0.2;
    }
    moto_control.angular.z=angle/(360)*abs(n)*0.04;
    if(moto_control.angular.z>0.3)moto_control.angular.z=0.3;
    if(moto_control.angular.z<-0.3)moto_control.angular.z=-0.3;

    m_lock.lock();
    if(!stop_flag){
      ROS_INFO("linear: %.3lf, angular: %.3lf", moto_control.linear.x, moto_control.angular.z);
      twist_pub.publish(moto_control);
    }
    m_lock.unlock();
  }

  void pose_track(const nav_msgs::OdometryConstPtr &odometry_msg) {
    Point3f teachPoint, odomPoint;
    float pointDistance, minDistance = 1000;
    odomPoint.x = odometry_msg->pose.pose.position.x;
    odomPoint.y = odometry_msg->pose.pose.position.y;
    odomPoint.z = odometry_msg->pose.pose.position.z;
    Eigen::Quaterniond odomPose(odometry_msg->pose.pose.orientation.w,odometry_msg->pose.pose.orientation.x,
                         odometry_msg->pose.pose.orientation.y,odometry_msg->pose.pose.orientation.z);//w,x,y,z

/**********************在示教路径中找到一个与当前位置最近的点**********************/
    if (initialCount < 10) //前十次的滤波值舍弃，让其逼近真实值
    {
      initialCount++;
      if (initialCount == 10) { //先找一个最近的点，逐次逼近
        for (int i = 0; i < rowcount; i++) {
          teachPoint.x = teachPath.at<float>(i, 1);
          teachPoint.y = teachPath.at<float>(i, 2);
          teachPoint.z = teachPath.at<float>(i, 3);
          pointDistance = calculateDistance(teachPoint, odomPoint);
          if (pointDistance < minDistance) {
            minDistance = pointDistance;
            currentPositon = i;
          } //找到距离最近的那个点
        }
      }
    }
    else {
      if (lastPosition < 50) 
      //从最後面选rowcount+(lastPosition-50)个，从前面选0~lastPosition+50个，这样正好100个点
      {
        for (int i = rowcount + (lastPosition - 50); i < rowcount; i++) {
          teachPoint.x = teachPath.at<float>(i, 1);
          teachPoint.y = teachPath.at<float>(i, 2);
          teachPoint.z = teachPath.at<float>(i, 3);
          pointDistance = calculateDistance(teachPoint, odomPoint);
          if (pointDistance < minDistance) {
            minDistance = pointDistance;
            currentPositon = i;
          } //找到距离最近的那个点
        }
        for (int i = 0; i < lastPosition + 50; i++) {
          teachPoint.x = teachPath.at<float>(i, 1);
          teachPoint.y = teachPath.at<float>(i, 2);
          teachPoint.z = teachPath.at<float>(i, 3);
          pointDistance = calculateDistance(teachPoint, odomPoint);
          if (pointDistance < minDistance) {
            minDistance = pointDistance;
            currentPositon = i;
          } //找到距离最近的那个点
        }
      } else {
        for (int i = lastPosition - 50; i < lastPosition + 50; i++) {
          if (i >= rowcount) {
            i = 0;
            lastPosition -= rowcount;
          }
          teachPoint.x = teachPath.at<float>(i, 1);
          teachPoint.y = teachPath.at<float>(i, 2);
          teachPoint.z = teachPath.at<float>(i, 3);
          pointDistance = calculateDistance(teachPoint, odomPoint);
          if (pointDistance < minDistance) {
            minDistance = pointDistance;
            currentPositon = i;
          } //找到距离最近的那个点
        }
      }
/*******************到这里结束，找到点的位置为currentPosition*******************/
/********下面就要将currentPositon的三维坐标以marker点的形式显示到示教路径中********/
      if(forward_flag){
        targetPosition=rowcount-500;
        if(targetPosition-currentPositon>foresee)n=foresee;
        else n=targetPosition-currentPositon;
        if(n<=0){forward_flag=!forward_flag;speedcount=0;}
      }
      else{
        targetPosition=100;
        if(currentPositon-targetPosition>foresee)n=-foresee;
        else n=targetPosition-currentPositon;
        if(n>=0){forward_flag=!forward_flag;speedcount=0;}
      }

      geometry_msgs::Point point;
      teachPoint.x=teachPath.at<float>(currentPositon+n,1);
      teachPoint.y=teachPath.at<float>(currentPositon+n,2);
      teachPoint.z=teachPath.at<float>(currentPositon+n,3);
      point.x=teachPoint.x;point.y=teachPoint.y;point.z=teachPoint.z;
      pub_marker(teachPoint_pub, point, odometry_msg->header, 0, 1.0, 1.0, 0.0);

      cv::Point3f targetPoint(teachPath.at<float>(targetPosition,1),
                              teachPath.at<float>(targetPosition,2),
                              teachPath.at<float>(targetPosition,3));
      point.x=targetPoint.x;point.y=targetPoint.y;point.z=targetPoint.z;
      pub_marker(targetPoint_pub,point, odometry_msg->header, 1, 1.0, 0.0, 0.0);

      navigation(odomPoint,odomPose,teachPoint);
    }
    lastPosition = currentPositon;
  }

  void readFile_() {

    ReadFile readfile;
    const char *filename = path_file.c_str();
    rowcount = readfile.getFileRows(filename);
    if (!rowcount) {
      ROS_WARN("Path file does not exist!");
      return;
    }
    colcount = readfile.getFileColumns(filename);
    if (!readfile.LoadData(filename, teachPath, rowcount, colcount, 1)) {
      ROS_WARN("Failed to read file!");
      return;
    }
    for (int i = 0; i < rowcount; i++) {
      this_pose_stamped.header.stamp = ros::Time::now();
      this_pose_stamped.header.frame_id = "odom";
      this_pose_stamped.pose.position.x = teachPath.at<float>(i, 1);
      this_pose_stamped.pose.position.y = teachPath.at<float>(i, 2);
      this_pose_stamped.pose.position.z = teachPath.at<float>(i, 3);
      path.poses.push_back(this_pose_stamped);
    }
  }

  void pub_marker(ros::Publisher publisher, geometry_msgs::Point &point, const std_msgs::Header &header,
                  int32_t marker_id, float r, float g, float b){
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "point_shape";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = point;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    publisher.publish(marker);
  }

  void pub_path() { path_pub.publish(path); }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_tracker");

  pathTracker node;

  ros::Rate loop_rate(30);
  while (ros::ok()) {
    node.pub_path();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
