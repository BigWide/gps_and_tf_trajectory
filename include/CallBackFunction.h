#ifndef CALL_BACK_FUNCTION_H
#define CALL_BACK_FUNCTION_H

#include <ros/ros.h>
#include <fstream>
#include <tf2_msgs/TFMessage.h>
#include <npos220/npos220nav.h>
#include <pos320/Pos320Nav.h>
#include <xwgi5651/Xwgi5651Gpfpd.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


class CallBackFunction
{
    public:
        CallBackFunction(Eigen::Matrix3d R_, Eigen::Vector3d t_);
        ~CallBackFunction();

        void tf_call_back(const tf2_msgs::TFMessageConstPtr &msg);
        void nav_call_back(const npos220::npos220navConstPtr &msg);
        // void lidar_call_back(const tf2_msgs::TFMessage::ConstPtr &msg);
        // void camera_call_back(const tf2_msgs::TFMessage::ConstPtr &msg);
        


    private:
        int marker1_id, marker2_id;
        bool one;
        bool IsSave;
        double lat;//纬度
        double lon;//经度
        double alt;//高程
        double R_N;//卯酉圈曲率半径
        double R_M;//子午圈曲率半径
        double R_E;
        double kkk; 
        const double e;//参考椭球体扁率(Re-Rp)/Re;
        const double R_0;
        const double Re;//参考椭球体长半轴  
        const double Rp;//参考椭球体短半轴 
        const double IPI;//3.1415926535898/180.0

        Eigen::Matrix3d Rie;//ENU坐标系到组合导航坐标系的旋转矩阵
        //旋转组合导航轨迹,若有两个传感器之间的外参标定，则直接使用标定信息
        Eigen::Vector3d ea0;//组合导航坐标系到局部坐标系(其他传感器坐标系)的旋转欧拉角，一般通过标定获得
        Eigen::Vector3d tt;//组合导航坐标系到局部坐标系(其他传感器坐标系)的平移向量，一般通过标定获得
        Eigen::Matrix3d R;//组合导航坐标系到局部坐标系(其他传感器坐标系)的旋转矩阵

        Eigen::Vector3d EARTH_pos;//第一帧在地球直角坐标系下的位置
        Eigen::Vector3d ENU_pos;//在ENU坐标系下的位置
        Eigen::Vector3d NAV_pos;//在组合导航坐标系下的位置
        Eigen::Vector3d LOCAL_pos;//在局部坐标系(其他传感器坐标系)下的位置

        ros::NodeHandle nh;
        ros::Subscriber tf_sub;
        ros::Subscriber nav_sub;
        ros::Subscriber lidar_sub;
        ros::Subscriber camera_sub;
        ros::Publisher nav_trajectory;
        ros::Publisher tf_trajectory;

};






#endif /* CALLBACK_FUNCTION_H */