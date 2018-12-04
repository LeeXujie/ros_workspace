#include "cmath"
#include "fstream"
#include "iostream"
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "creat_path");
  ros::NodeHandle nh("~");

  string file_path;
  nh.param<std::string>("file_path", file_path, "navigation_path.txt");

  float i = 1.0;
  float x = -9.0, y = 0.0, z = 2.0;

  ofstream write_file(file_path);

  for (; i <= 500; i++) {
    x = x + 5.0 / 500;
    write_file << i << " " << x << " " << y << " " << z << endl;
  }
  for (; i <= 700; i++) {
    x = x + 2.0 / 200;
    z = z - 0.5 / 200;
    write_file << i << " " << x << " " << y << " " << z << endl;
  }
  for (; i <= 900; i++) { //右下弧
    x = x + 1.0 / 200;
    z = 2.5 - sqrt(1 - (i - 700) * 1.0 / 200 * (i - 700) * 1.0 / 200);
    write_file << i << " " << x << " " << y << " " << z << endl;
  }
  for (; i <= 1050; i++) {
    z = z + 1.5 / 150;
    write_file << i << " " << x << " " << y << " " << z << endl;
  }


  // for (; i < 800; i++) { //右上弧
  //   z = 3.5 + (i - 600) * 0.5 / 200;
  //   y = 0.5 + sqrt(0.25 - ((i - 600) * 0.5 / 200) * ((i - 600) * 0.5 / 200));
  //   write_file << i << " " << x << " " << y << " " << z << endl;
  // }
  // for (; i < 1200; i++) {
  //   y = 0.5 - (i - 800) * 2 / 400;
  //   z = 4.0;
  //   write_file << i << " " << x << " " << y << " " << z << endl;
  // }
  // for (; i < 1400; i++) { //左上弧
  //   y = -1.5 - (i - 1200) * 0.5 / 200;
  //   z = 3.5 + sqrt(0.25 - ((i - 1200) * 0.5 / 200) * ((i - 1200) * 0.5 / 200));
  //   write_file << i << " " << x << " " << y << " " << z << endl;
  // }
  // for (; i < 1600; i++) {
  //   y = -2.0;
  //   z = 3.5 - (i - 1400) * 1 / 200;
  //   write_file << i << " " << x << " " << y << " " << z << endl;
  // }
  // for (; i < 1800; i++) { //左下弧
  //   z = 2.5 - (i - 1600) * 0.5 / 200;
  //   y = -1.5; //- sqrt(0.25 - ((i - 1600)*0.5 / 200)*((i - 1600)*0.5 / 200));
  //   write_file << i << " " << x << " " << y << " " << z << endl;
  // }
  // for (; i < 2000; i++) {
  //   y = -1.5 + (i - 1800) * 1 / 200;
  //   z = 2.0;
  //   write_file << i << " " << x << " " << y << " " << z << endl;
  // }

  /*for (; i < 715; i++){
          y = -2 + i * 3 / 715;
          z = 2.0;
          write_file << i << " " << x << " " << y << " " << z << endl;
  }
  for (; i < 1000; i++){
          y = 1.0;
          z = 2.0 + (i - 715) * 2 / 285;
          write_file << i << " " << x << " " << y << " " << z << endl;
  }
  for (; i < 1715; i++){
          y = 1.0 - (i - 1000) * 3 / 715;
          z = 4.0;
          write_file << i << " " << x << " " << y << " " << z << endl;
  }
  for (; i < 2000; i++){
          y = -2.0;
          z = 4.0 - (i - 1715) * 2 / 285;
          write_file << i << " " << x << " " << y << " " << z << endl;
  }*/
  // system("pause");
  ROS_INFO_STREAM("File has been saved to "+file_path);
  ros::shutdown();
  return 0;
}
