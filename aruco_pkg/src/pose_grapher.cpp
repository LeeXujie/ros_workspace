//加载二维码分布图的pcd文件，以点云的形式发布出去；
//订阅fusion_filter滤波后的里程计信息，发布相机的位姿信息以及路径信息

#include "CameraPoseVisualization.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

class poseGrapher {
private:
  ros::NodeHandle nh_;
  nav_msgs::Path path;
  ros::Subscriber sub;
  ros::Publisher pcl_pub;
  ros::Publisher path_pub;
  ros::Publisher markerArray_pub;
  CameraPoseVisualization camera;

  std::string pcd_path;
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ> cloud;

public:
  poseGrapher() : nh_("~"), camera(1, 0, 0, 1) {
    nh_.param<std::string>("pcd_path", pcd_path, "map.pcd");
    path.header.stamp = ros::Time(0);
    path.header.frame_id = "odom";
    pcl::io::loadPCDFile(pcd_path, cloud);
    // Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";
    pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("pcl_markermap", 1, true);
    path_pub = nh_.advertise<nav_msgs::Path>("filtered_trajectory", 10, true);
    markerArray_pub = nh_.advertise<visualization_msgs::MarkerArray>("camera_marker", 10);
    sub = nh_.subscribe("/vo_data", 10, &poseGrapher::pose_graph, this);
  }

  void pose_graph(const nav_msgs::OdometryConstPtr &odometry_msg) {

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header = odometry_msg->header;
    this_pose_stamped.pose.position = odometry_msg->pose.pose.position;
    path.poses.push_back(this_pose_stamped);
    path_pub.publish(path);

    visualization_msgs::Marker cam_marker;
    cam_marker.header = odometry_msg->header;
    Eigen::Vector3d vio_t_cam(odometry_msg->pose.pose.position.x,
                              odometry_msg->pose.pose.position.y,
                              odometry_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q_cam(odometry_msg->pose.pose.orientation.w,
                                 odometry_msg->pose.pose.orientation.x,
                                 odometry_msg->pose.pose.orientation.y,
                                 odometry_msg->pose.pose.orientation.z);
    camera.reset();
    camera.add_pose(vio_t_cam, vio_q_cam);
    camera.publish_by(markerArray_pub, cam_marker.header);
  }

  void pub_map() { pcl_pub.publish(output); }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "pose_grapher_node");

  poseGrapher node;

  ros::Rate loop_rate(30);
  while (ros::ok()) {
    node.pub_map();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
