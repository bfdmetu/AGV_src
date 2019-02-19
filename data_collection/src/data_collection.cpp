//
//  Sutpc_ros_odom.cpp
//  sutpc_ros_odom
//
//  Created by 钟海兴 on 2018/02/19.
//  Copyright © 2018 钟海兴. All rights reserved.
//

#include "data_collection.hpp"
#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/UInt8MultiArray.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <time.h>
#include <math.h>
#include "sensor_msgs/Imu.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <visualization_msgs/MarkerArray.h>




void odomcallback(const OdomConstPtr& odom)
{
    ROS_INFO("odom x : %f , odom y : %f , odom z : %f",odom->pose.pose.position.x , odom->pose.pose.position.y , odom->pose.pose.orientation.w);

    //ROS_INFO("x pos :  , y pos : );


}

void imucallback(const ImuConstPtr& imu)
{
    ROS_INFO("imu x : %f , imu y : %f , imu w: %f",imu->orientation.x , imu->orientation.y , imu->orientation.w);

    //ROS_INFO("x pos :  , y pos : );
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_collection");
    ros::NodeHandle n;
    ROS_INFO("data collecting...");

    ros::Subscriber sub_odom = n.subscribe<nav_msgs::Odometry>("odom", 10, odomcallback);
    ros::Subscriber sub_imu = n.subscribe<sensor_msgs::Imu>("imu", 10, imucallback);


    ros::spin();

    return 0;
}
