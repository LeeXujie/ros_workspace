#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <aruco/aruco.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <mutex>
#include <queue>
#include <ros/time.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace cv;

char cvKey;
int waitTime=5;

std::mutex m_buf;
queue<cv::Mat> T_buf;
queue<ros::Time> stamp_T_buf;
vector<vector<KeyPoint>> keyPoints_buf;
vector<cv::Mat> descriptors_buf;
vector<ros::Time> stamp_img_buf;

Mat T_1,T_2;//1 is current, 2 is previous
Mat image_1,image_2,img_match;
Mat descriptors_1, descriptors_2;
vector<KeyPoint> keyPoints_1, keyPoints_2;
Ptr<ORB> orb = ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::FAST_SCORE, 31, 20);

ros::Publisher pcl_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

void poseCallback(const nav_msgs::OdometryConstPtr &odometry_msg);
void imageCallback(const sensor_msgs::ImageConstPtr &msg);
Point2f pixel2cam( const Point2d& p, const Mat& K );

bool match_two_images (
    cv::Mat descriptors1,
    cv::Mat descriptors2,
    std::vector< DMatch >& matches );

void triangulation (
    const vector<KeyPoint>& keypoint_1,
    const vector<KeyPoint>& keypoint_2,
    const std::vector< DMatch >& matches,
    const Mat& T1, const Mat& T2,
    vector<Point3d>& points
    );


int main ( int argc, char** argv )
{
  ros::init(argc, argv, "triangulation_node");
  ros::NodeHandle nh("~");
  if(!nh.ok())return 0;

  image_transport::ImageTransport it(nh);
  ros::Subscriber sub_image = nh.subscribe("image_raw", 10, imageCallback);
  ros::Subscriber sub_pose = nh.subscribe("vo_data", 10, poseCallback);
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);

  ros::Rate loop_rate(30);

  while (ros::ok()) {
    if (cvKey == 27) {
      cv::destroyAllWindows();
      ros::shutdown();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void poseCallback(const nav_msgs::OdometryConstPtr &odometry_msg){
  Eigen::Quaterniond Q(odometry_msg->pose.pose.orientation.w,odometry_msg->pose.pose.orientation.x,
                       odometry_msg->pose.pose.orientation.y,odometry_msg->pose.pose.orientation.z);//w,x,y,z
  Eigen::Matrix3d RR = Q.matrix();
  cv::Mat tt = (Mat_<float>(3,1) << odometry_msg->pose.pose.position.x,
                odometry_msg->pose.pose.position.y,odometry_msg->pose.pose.position.z);
  cv::Mat T44=(Mat_<float>(4,4) <<
               RR(0,0), RR(0,1), RR(0,2), tt.at<float>(0,0),
               RR(1,0), RR(1,1), RR(1,2), tt.at<float>(1,0),
               RR(2,0), RR(2,1), RR(2,2), tt.at<float>(2,0),
               0,       0,       0,       1             );
  cv::Mat TInv=T44.inv();
  cv::Mat T34=(Mat_<float>(3,4) <<
               TInv.at<float>(0,0), TInv.at<float>(0,1), TInv.at<float>(0,2), TInv.at<float>(0,3),
               TInv.at<float>(1,0), TInv.at<float>(1,1), TInv.at<float>(1,2), TInv.at<float>(1,3),
               TInv.at<float>(2,0), TInv.at<float>(2,1), TInv.at<float>(2,2), TInv.at<float>(2,3));

  m_buf.lock();
  if(T_buf.size()>100)
  {
    T_buf.pop();
    stamp_T_buf.pop();
  }
  T_buf.push(T34.clone());
  stamp_T_buf.push(odometry_msg->header.stamp);
  m_buf.unlock();
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  if (cvKey != 27) {
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    if(!img.isContinuous())return;

    vector<KeyPoint> keyPoints;
    orb->detect(img, keyPoints);
    if (keyPoints.empty())return;

    Mat descriptors;
    orb->compute(img, keyPoints, descriptors);

    if(keyPoints_buf.size()>100)
    {
      keyPoints_buf.erase(keyPoints_buf.begin());
      descriptors_buf.erase(descriptors_buf.begin());
      stamp_img_buf.erase(stamp_img_buf.begin());
    }
    keyPoints_buf.push_back(keyPoints);
    descriptors_buf.push_back(descriptors.clone());
    stamp_img_buf.push_back(msg->header.stamp);

    m_buf.lock();
    if(!stamp_T_buf.empty())
    {
      for(int i=0;i<stamp_img_buf.size();i++)
      {
        if(stamp_img_buf.at(i)==stamp_T_buf.back())//if have the same stamps
        {
          T_1=T_buf.back();
          image_1=img.clone();
          keyPoints_1=keyPoints_buf[i];
          descriptors_1=descriptors_buf[i].clone();
          keyPoints_buf.erase(keyPoints_buf.begin(),keyPoints_buf.begin()+i);
          descriptors_buf.erase(descriptors_buf.begin(),descriptors_buf.begin()+i);
          stamp_img_buf.erase(stamp_img_buf.begin(),stamp_img_buf.begin()+i);
          break;
        }
      }
      stamp_T_buf.pop();
      T_buf.pop();
    }
    m_buf.unlock();

    if(!T_2.empty())
    {
      vector<DMatch> matches;
      if (match_two_images(descriptors_1, descriptors_2, matches))//If descriptors_1==descriptors_2, it will return false, because the hamming-distance is 0.
      {
        vector<Point3d> points;
        triangulation(keyPoints_1, keyPoints_2, matches, T_1, T_2, points);
        pcl::PointXYZ basic_point;
        sensor_msgs::PointCloud2 clouds;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for ( int i=0; i<matches.size(); i++ )
        {
          basic_point.x = points[i].x;
          basic_point.y = points[i].y;
          basic_point.z = points[i].z;
          cloud.points.push_back(basic_point);
//          cout<<"point in the world frame: "<<points[i]<<endl;
        } cout<<endl;
        cout<<"T_1="<<T_1<<endl<<"T_2="<<T_2<<endl;

        pcl::toROSMsg(cloud, clouds);
        clouds.header.frame_id = "odom";
        clouds.header.stamp = ros::Time::now();
        pcl_pub.publish(clouds);

        drawMatches(image_1,keyPoints_1,image_2,keyPoints_2,matches,img_match,Scalar(0,255,0),Scalar(0,255,0),std::vector<char>(),DrawMatchesFlags::DEFAULT);
        imshow("ORB matches", img_match);
      }
    }

    T_2=T_1.clone();
    keyPoints_2=keyPoints_1;
    descriptors_2=descriptors_1.clone();
    image_2=image_1.clone();

    drawKeypoints(img,keyPoints,img,Scalar(0, 255, 0),DrawMatchesFlags::DEFAULT);
    imshow("ORB feature points", img);
    cvKey = cv::waitKey(waitTime);
    if (cvKey == 's') waitTime = waitTime == 0 ? 5 : 0;
  }
}

bool match_two_images ( cv::Mat descriptors1, cv::Mat descriptors2, std::vector< DMatch >& matches )
{
  std::vector<DMatch> match;
  Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
  matcher->match ( descriptors_1, descriptors_2, match );

  double dist=0,avrgDist=0;
  for ( int i = 0; i < descriptors_1.rows; i++ )
  {
    dist += match[i].distance;
  }
  avrgDist=dist/(descriptors_1.rows+1);
  if(avrgDist==0)return false;

  for ( int i = 0; i < descriptors_1.rows; i++ )
  {
    if ( match[i].distance <= min ( avrgDist, 30.0 ) )
    {
      matches.push_back ( match[i] );
    }
  }

  if(matches.empty())return false;
  return true;
}

void triangulation (
    const vector< KeyPoint >& keypoint_1,
    const vector< KeyPoint >& keypoint_2,
    const std::vector< DMatch >& matches,
    const Mat& T1, const Mat& T2,
    vector< Point3d >& points )
{
  Mat K = ( Mat_<double> ( 3,3 ) << 1181.221956756576, 0, 334.8271945796204, 0, 1177.892891376225, 245.1326109540756, 0, 0, 1 );
  vector<Point2f> pts_1, pts_2;
  for ( DMatch m:matches )
  {
    // 将像素坐标转换至相机坐标
    pts_1.push_back ( pixel2cam( keypoint_1[m.queryIdx].pt, K) );
    pts_2.push_back ( pixel2cam( keypoint_2[m.trainIdx].pt, K) );
  }

  Mat pts_4d;
  cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );

  // 转换成非齐次坐标
  for ( int i=0; i<pts_4d.cols; i++ )
  {
    Mat x = pts_4d.col(i);
    x /= x.at<float>(3,0); // 归一化
    Point3d p (
          x.at<float>(0,0),
          x.at<float>(1,0),
          x.at<float>(2,0)
          );
    points.push_back( p );
  }
}

Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
  return Point2f (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0),
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1)
        );
}

