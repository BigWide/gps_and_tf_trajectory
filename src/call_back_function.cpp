#include <iostream>
#include <fstream>
#include <cmath>
#include <pos320/Pos320Nav.h>
#include <npos220/npos220nav.h>
#include <xwgi5651/Xwgi5651Gpfpd.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <rosgraph_msgs/Clock.h>
#include <gps_and_tf_trajectory/tf_image.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include "CallBackFunction.h"


CallBackFunction::CallBackFunction(Eigen::Matrix3d R_, Eigen::Vector3d t_):marker1_id(0), marker2_id(0), one(true), e(0.0818191908425), R_0(6378137), Re(6378137), Rp(6356752.3), 
                                                                           IPI(0.017453292), R(R_), tt(t_)
{
    // std::string IsSaveTrajectory, TfFileName, NavFileName;
    // std::cin>>IsSaveTrajectory;
    // if(IsSaveTrajectory=="Y"||IsSaveTrajectory=="y")
    // {
    //   IsSave = true;
    //   std::cout<<"请输入tf轨迹文件名"<<std::endl;
    //   std::cin>>TfFileName;
    //   std::cout<<"请输入nav轨迹文件名"<<std::endl;
    //   std::cin>>NavFileName;
    //   std::ofstream txtfile;
    //   txtfile.open(TfFileName);
    // }
    // else
    // {
    //   IsSave = false;
    // }
    
    tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1, &CallBackFunction::tf_call_back,this);
    nav_sub = nh.subscribe<npos220::npos220nav>("/npos220_nav", 1, &CallBackFunction::nav_call_back,this);
    // lidar_sub = nh.subscribe<tf2_msgs::TFMessage>("/pandar_points", 1, lidar_call_back);
    // camera_sub = nh.subscribe<tf2_msgs::TFMessage>("/kbd_camera", 1, camera_call_back);
    nav_trajectory = nh.advertise<visualization_msgs::Marker>("/GPS_trajectory", 1, true);
    tf_trajectory = nh.advertise<visualization_msgs::Marker>("/tf_trajectory", 1, true);
}

CallBackFunction::~CallBackFunction(){}

void CallBackFunction::tf_call_back(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    visualization_msgs::Marker marker2;
      marker2.header.frame_id = "map";
      marker2.header.stamp = msg->transforms[0].header.stamp;
      marker2.ns = "tf_trajectory";
      marker2.id = marker2_id++;
      marker2.type = visualization_msgs::Marker::CUBE;
      marker2.action = visualization_msgs::Marker::ADD;
      marker2.pose.position.x = msg->transforms[0].transform.translation.x;
      marker2.pose.position.y = msg->transforms[0].transform.translation.y;
      marker2.pose.position.z = 0;
      marker2.pose.orientation.x = 0.0;
      marker2.pose.orientation.y = 0.0;
      marker2.pose.orientation.z = 0.0;
      marker2.pose.orientation.w = 1.0;
      marker2.scale.x = 0.05;
      marker2.scale.y = 0.05;
      marker2.scale.z = 0.05;
      marker2.color.r = 1.0f;
      marker2.color.g = 0.0f;
      marker2.color.b = 0.0f;
      marker2.color.a = 1.0;
      marker2.lifetime = ros::Duration();
      tf_trajectory.publish(marker2);
}

void CallBackFunction::nav_call_back(const npos220::npos220nav::ConstPtr &msg)
{
    lat=msg->lat*IPI;
    lon=msg->lon*IPI;
    alt=msg->alt;
    Eigen::Vector3d eulerAngle(msg->heading*IPI,msg->pitch*IPI,msg->roll*IPI);
    //以第一帧为坐标系原点
    if(one)
    {
      one=false;
      kkk=sqrt(1-(e*sin(lat)*e*sin(lat)));
      R_E=R_0/kkk;
      EARTH_pos[0]=(R_E+alt)*cos(lat)*cos(lon);
      EARTH_pos[1]=(R_E+alt)*cos(lat)*sin(lon);
      EARTH_pos[2]=(R_E*(1-e*e)+alt)*sin(lat);
      ENU_pos[0]=0;
      ENU_pos[1]=0;
      ENU_pos[2]=0;
      //计算ENU坐标系到组合导航坐标系的变换矩阵(旋转矩阵)，因为组合导航坐标系会随着组合导航的移动而发生改变，所以一般以第一帧的组合导航的坐标系为组合导航坐标系
      Eigen::Vector3d eulerAngle(msg->heading*IPI,msg->pitch*IPI,msg->roll*IPI);
      Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
      Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
      Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));
      Eigen::Quaterniond q_nav;
      q_nav=yawAngle*pitchAngle*rollAngle;
      Rie = q_nav.matrix();//ENU坐标系到IMU坐标系的旋转矩阵
    }
    else
    {
      //将经纬度信息转换到地球直角坐标系下的坐标(原点位于球心，z轴与地球自转轴重合，正向沿地球自传方向，x轴和y轴位于赤道平面内，x轴穿过本初子午线，y轴穿过90°子午线)
      double dx=(R_E+alt)*cos(lat)*cos(lon)-EARTH_pos[0];
      double dy=(R_E+alt)*cos(lat)*sin(lon)-EARTH_pos[1];
      double dz=(R_E*(1-e*e)+alt)*sin(lat)-EARTH_pos[2];

      //转换到ENU(东北天)坐标系(ENU坐标系不会发生改变)
      ENU_pos[0]=-sin(lon)*dx+cos(lon)*dy;
      ENU_pos[1]=-sin(lat)*cos(lon)*dx-sin(lat)*sin(lon)*dy+dz*cos(lat);
      ENU_pos[2]=0;
      
      //将ENU坐标系下的坐标转换到组合导航坐标系
      NAV_pos = Rie*ENU_pos;

      //将ENU坐标系下的位置转换到相关局部坐标系下，与相关轨迹进行对齐
      LOCAL_pos=R*NAV_pos+tt;
    
      visualization_msgs::Marker marker1;
        marker1.header.frame_id = "map";
        marker1.header.stamp = ros::Time::now();
        marker1.ns = "gps_trajectory";
        marker1.id = marker1_id++;
        marker1.type = visualization_msgs::Marker::CUBE;
        marker1.action = visualization_msgs::Marker::ADD;
        marker1.pose.position.x = LOCAL_pos[0];
        marker1.pose.position.y = LOCAL_pos[1];
        marker1.pose.position.z = 0;
        marker1.pose.orientation.x = 0.0;
        marker1.pose.orientation.y = 0.0;
        marker1.pose.orientation.z = 0.0;
        marker1.pose.orientation.w = 1.0;
        marker1.scale.x = 0.05;
        marker1.scale.y = 0.05;
        marker1.scale.z = 0.001;
        marker1.color.r = 1.0f;
        marker1.color.g = 0.5f;
        marker1.color.b = 0.5f;
        marker1.color.a = 1.0;
        marker1.lifetime = ros::Duration();
        nav_trajectory.publish(marker1);
    }
}