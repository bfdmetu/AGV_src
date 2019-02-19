//
//  Sutpc_ros_odom.hpp
//  sutpc_ros_odom
//
//  Created by 钟海兴 on 2018/12/29.
//  Copyright © 2018 钟海兴. All rights reserved.
//

#ifndef data_collection_hpp
#define data_collection_hpp

#include     <stdio.h>
#include     <stdlib.h>
#include     <unistd.h>
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>
#include     <termios.h>
#include     <errno.h>

#include <boost/thread/mutex.hpp>

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef boost::shared_ptr<sensor_msgs::Imu const> ImuConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> VoConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> GpsConstPtr;
typedef boost::shared_ptr<geometry_msgs::Twist const> VelConstPtr;

#endif /* data_collection_hpp */
