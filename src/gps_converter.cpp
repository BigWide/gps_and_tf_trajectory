#include<iostream>
#include<fstream>
#include<cmath>
#include"CallBackFunction.h"




int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_converter");

  Eigen::Vector3d ea0, tt;
  ros::param::get("~yaw", ea0[0]);
  ros::param::get("~pitch", ea0[1]);
  ros::param::get("~roll", ea0[2]);
  ros::param::get("~tx", tt[0]);
  ros::param::get("~ty", tt[1]);
  ros::param::get("~tz", tt[2]);
  Eigen::Matrix3d R;
  

  R = Eigen::AngleAxisd(ea0[0]/180.0*3.1416,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(ea0[1]/180.0*3.1416,Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(ea0[2]/180.0*3.1416,Eigen::Vector3d::UnitX());
  CallBackFunction cbf(R, tt);
  ros::MultiThreadedSpinner ssp(2);
  ssp.spin();
  return 0;
}