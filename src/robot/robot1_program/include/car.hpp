#ifndef CAR_HPP
#define CAR_HPP

#define PI 3.1415926536
#include "string.h"  
#include "sstream"
#include "termio.h"
#include "iostream"
#include "fstream"
#include "vector"
#include "csignal"
#include "sys/time.h"
#include "ros/ros.h"  
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"
#include "gazebo_msgs/ModelStates.h"
#include "boost/thread.hpp"
#include "cmath"
#include "algorithm" 
#include "yaml-cpp/yaml.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

using namespace ros;
using namespace std;
using namespace sensor_msgs;

typedef struct 
{
    int sign; //1:automatic;0::manual;
    std_msgs::Float64 pub_left_string[4];
    std_msgs::Float64 pub_right_string[4];
    std_msgs::Float64 pub_left_wheel[4];
    std_msgs::Float64 pub_right_wheel[4];
}car_ctrl_;

typedef struct 
{
    cv::Point2d point;
    cv::Point2d point_boundary[2]; // 0 : x=0 ; 1 : x=width 
}wheel_;

typedef struct 
{
    double length;
    double width;  
    double laser_length;
    double laser_width;     
    double wheel_y[2];
    double wheel_x[4];

    double H;
    double L2;
    double k_v;
    double k_h;
    double k_a;

    cv::Point2d body_car[4];  //第一象限，逆时针
    wheel_ body_wheel[2][4];     //[i][j] : i=0,左轮，i=1,右轮
    cv::Point2d body_car_uv[4];     //第一象限，逆时针
    wheel_ body_wheel_uv[2][4];     //[i][j] : i=0,左轮，i=1,右轮

    vector<cv::Point2d> car_position;
}car_info_;

typedef struct{
    int obj_width;
    double Mat_scaling;
    double u0_x;
    double u0_y;
    double u_x;
    double u_y;
    cv::Mat Mat;
}img_data_;

void laser_callback(const sensor_msgs::LaserScanConstPtr& laser1, const sensor_msgs::LaserScanConstPtr & laser2,const sensor_msgs::LaserScanConstPtr & laser3 ,const sensor_msgs::LaserScanConstPtr & laser4);
car_ctrl_ cat_auto_ctrl(float distance_fl,float distance_fr,float distance_bl,float distance_br);
cv::Point2d Touv(cv::Point2d P0 , double u_x , double u_y , int u0 , int v0);
#endif