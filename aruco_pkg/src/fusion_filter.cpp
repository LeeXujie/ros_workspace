//订阅pose_estimator发布的里程计消息，对其进行滤波；
//将滤波后的里程计消息发布出去，再发布一个由odom到base_link的tf变换

#include <ros/ros.h>
#include "kalman_filter.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class fusionFilter {
private:
  Kalman kalman;
  KalmanInfo tx, ty, tz, qx, qy, qz, qw;

  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber sub_;
  tf::TransformBroadcaster odom_broadcaster;

  float Q,R;

public:
  fusionFilter() : nh_("~") {
    nh_.param<float>("Kalman_Q", Q, 1e-5);
    nh_.param<float>("Kalman_R", R, 1e-3);

    kalman.Init_KalmanInfo(&tx, 1e-5, 1e-3);
    kalman.Init_KalmanInfo(&ty, 1e-5, 1e-3);
    kalman.Init_KalmanInfo(&tz, 1e-5, 1e-3);
    kalman.Init_KalmanInfo(&qx, 1e-5, 1e-3);
    kalman.Init_KalmanInfo(&qy, 1e-5, 1e-3);
    kalman.Init_KalmanInfo(&qz, 1e-5, 1e-3);
    kalman.Init_KalmanInfo(&qw, 1e-5, 1e-3);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("vo_data", 10);
    sub_ = nh_.subscribe("/vo_data", 10, &fusionFilter::kalman_filter, this);
  }
  void kalman_filter(const nav_msgs::OdometryConstPtr &odometry_msg);
};

void fusionFilter::kalman_filter(const nav_msgs::OdometryConstPtr &odometry_msg) {
//  ROS_INFO("filtering......\n");

  kalman.m_KalmanFilter(&tx, odometry_msg->pose.pose.position.x);
  kalman.m_KalmanFilter(&ty, odometry_msg->pose.pose.position.y);
  kalman.m_KalmanFilter(&tz, odometry_msg->pose.pose.position.z);
  kalman.m_KalmanFilter(&qx, odometry_msg->pose.pose.orientation.x);
  kalman.m_KalmanFilter(&qy, odometry_msg->pose.pose.orientation.y);
  kalman.m_KalmanFilter(&qz, odometry_msg->pose.pose.orientation.z);
  kalman.m_KalmanFilter(&qw, odometry_msg->pose.pose.orientation.w);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header = odometry_msg->header;
  odom_trans.child_frame_id = odometry_msg->child_frame_id;
  odom_trans.transform.translation.x = tx.filterValue;
  odom_trans.transform.translation.y = ty.filterValue;
  odom_trans.transform.translation.z = tz.filterValue;
  odom_trans.transform.rotation = odometry_msg->pose.pose.orientation;
  odom_trans.transform.rotation.w = -odometry_msg->pose.pose.orientation.w;
  odom_broadcaster.sendTransform(odom_trans);

  nav_msgs::Odometry odom;
  odom.header = odometry_msg->header;
  odom.child_frame_id = odometry_msg->child_frame_id;
  odom.pose.pose.position.x = tx.filterValue;
  odom.pose.pose.position.y = ty.filterValue;
  odom.pose.pose.position.z = tz.filterValue;
  odom.pose.pose.orientation.x = qx.filterValue;
  odom.pose.pose.orientation.y = qy.filterValue;
  odom.pose.pose.orientation.z = qz.filterValue;
  odom.pose.pose.orientation.w = qw.filterValue;
  odom_pub_.publish(odom);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fusion_filter");

  fusionFilter node;

  ros::spin();

  return 0;
}
