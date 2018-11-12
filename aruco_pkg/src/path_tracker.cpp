//加载示教路径点文件，将其发布出去显示到RViz
//订阅fusion_filter滤波后的里程计消息，在示教路径点中查找与当前相机位姿对应的点

#include "ReadFile.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

float calculateDistance(Point3f pt1, Point3f pt2) {
  return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) +
              (pt1.y - pt2.y) * (pt1.y - pt2.y) +
              (pt1.z - pt2.z) * (pt1.z - pt2.z));
}

class pathTracker {
private:
  ros::NodeHandle nh;
  nav_msgs::Path path;
  ros::Subscriber sub;
  ros::Publisher path_pub;
  ros::Publisher marker_pub;
  std::string path_file;
  geometry_msgs::PoseStamped this_pose_stamped;

  Mat teachPath;
  int rowcount, colcount;
  int initialCount, currentPositon, lastPosition;

public:
  pathTracker() : nh("~") {
    nh.param<std::string>("path_file", path_file, "navigation_path.txt");

    path.header.frame_id = "odom";
    path.header.stamp = ros::Time(0);
    path_pub = nh.advertise<nav_msgs::Path>("teaching_path", 10, true);
    marker_pub = nh.advertise<visualization_msgs::Marker>("teaching_point", 10);
    sub = nh.subscribe("/vo_data", 10, &pathTracker::pose_track, this);

    readFile_();
    initialCount = 0;
  }

  void pose_track(const nav_msgs::OdometryConstPtr &odometry_msg) {
    Point3f teachPoint, odomPoint;
    float pointDistance, minDistance = 1000;
    odomPoint.x = odometry_msg->pose.pose.position.x;
    odomPoint.y = odometry_msg->pose.pose.position.y;
    odomPoint.z = odometry_msg->pose.pose.position.z;

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
      geometry_msgs::Point point;
      point.x = teachPath.at<float>(currentPositon, 1);
      point.y = teachPath.at<float>(currentPositon, 2);
      point.z = teachPath.at<float>(currentPositon, 3);
      pub_marker(point, odometry_msg->header);
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

  void pub_path() { path_pub.publish(path); }

  void pub_marker(geometry_msgs::Point &point, const std_msgs::Header &header){
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "point_shape";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = point;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
  }
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
