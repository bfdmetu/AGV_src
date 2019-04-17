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


#include <sensor_msgs/NavSatFix.h>

#include <fstream>
using namespace std;

#include <sensor_msgs/PointCloud2.h>

// //NDT include
// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/registration/ndt.h>
// #include <pcl/filters/approximate_voxel_grid.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/visualization/pcl_visualizer.h>

#define save_data_to_file
#define save_data_to_file_GPS

double odom_x,odom_y;
double imu_roll, imu_pitch, imu_yaw;
double odom_roll, odom_pitch, odom_yaw;
int data_time_stamps = 0;

double gps_lo,gps_la;

// point cloud NDT param
char first_point_cloud = 0;
// pcl::PointCloud<pcl::PointXYZ> scan_1;
// pcl::PointCloud<pcl::PointXYZ> map_1;

void odomcallback(const OdomConstPtr& odom)
{

    std::string date_type = "odom";
    double timestamps_mark;

    timestamps_mark = odom->header.seq;
    odom_x = odom->pose.pose.position.x;
    odom_y = odom->pose.pose.position.y;

    tf::Quaternion quat;
	  tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);

	  //tf::Matrix3x3().getRPY()函数将四元数转换为rpy(分别为绕xyz)
	  tf::Matrix3x3(quat).getRPY(odom_roll, odom_pitch, odom_yaw);

    #ifdef save_data_to_file
    ofstream outfile;
    outfile.open("data.txt", ios::binary | ios::app | ios::in | ios::out);
    if(!outfile) cout<<"error"<<endl;

    outfile<<"Timestamps:"<< data_time_stamps << "," <<"senser_type:"<< "odom_pos" <<","<<"x:"<<odom_x<<","<<"y:"<< odom_y <<"\n";

    outfile.close();//关闭文件，保存文件。
    ROS_INFO("save data row : %f",timestamps_mark);
    #endif

}

void imucallback(const ImuConstPtr& imu)
{

    //ROS_INFO("x pos :  , y pos : );

    tf::Quaternion quat;
	  tf::quaternionMsgToTF(imu->orientation, quat);

	  //tf::Matrix3x3().getRPY()函数将四元数转换为rpy(分别为绕xyz)
	  tf::Matrix3x3(quat).getRPY(imu_roll, imu_pitch, imu_yaw);
}

void gpscallback(const sensor_msgs::NavSatFixConstPtr& gps_ori)
{
    gps_lo = gps_ori->longitude;
    gps_la = gps_ori->latitude;

}

// void visualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud){
//     // Initializing point cloud visualizer
//     boost::shared_ptr<pcl::visualization::PCLVisualizer>
//             viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//     viewer_final->setBackgroundColor (0, 0, 0);
//
//     // Coloring and visualizing target cloud (red).
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//             target_color (target_cloud, 255, 0, 0);
//     viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
//     viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                     1, "target cloud");
//
//     // Coloring and visualizing transformed input cloud (green).
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//             output_color (output_cloud, 0, 255, 0);
//     viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
//     viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                     1, "output cloud");
//
//     // Starting visualizer
//     viewer_final->addCoordinateSystem (1.0, "global");
//     viewer_final->initCameraParameters ();
//
//     // Wait until visualizer window is closed.
//     while (!viewer_final->wasStopped ())
//     {
//         viewer_final->spinOnce (100);
//         boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//     }
// }
//
// static void pc_callback(const sensor_msgs::PointCloud2::ConstPtr& input){
//
//     if ( first_point_cloud == 0 ) {
//       //once enough
//       first_point_cloud = 1;
//       pcl::fromROSMsg(*input, map_1);
//       std::cout << "Loaded " << map_1.size() << " data points from scan cloud" << std::endl;
//
//       return;
//     }
//     else if ( first_point_cloud == 1 && data_time_stamps > 1) {
//       //once enough
//       first_point_cloud = 2;
//       pcl::fromROSMsg(*input, scan_1);
//
//       //载入点云
//       std::cout << "Loaded " << scan_1.size() << " data points from scan cloud" << std::endl;
//       std::cout << "Loaded " << map_1.size() << " data points from map cloud" << std::endl;
//
//
//       //过滤扫描点云
//       pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan_1));
//       pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map_1));
//       pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//       pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
//       approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
//
//       approximate_voxel_filter.setInputCloud(scan_ptr);
//       approximate_voxel_filter.filter(*filtered_cloud);
//
//       std::cout<<"Filtered cloud contains "<< filtered_cloud->size() << "data points from scan cloud" << std::endl;
//
//       pcl::io::savePCDFileASCII("scan_1.pcd", scan_1);
//       pcl::io::savePCDFileASCII("map_1.pcd", map_1);
//       std::cout<<"pcd files save !!"<< std::endl;
//
//       //初始化NDT参数
//       pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
//       ndt.setTransformationEpsilon(0.01);
//       ndt.setStepSize(0.1);
//       ndt.setResolution(1.0);
//
//       ndt.setMaximumIterations(35);
//       ndt.setInputSource(filtered_cloud);
//       ndt.setInputTarget(map_ptr);
//
//       Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
//       Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
//       Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
//
//       pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//       //开始配准
//       ndt.align(*output_cloud, init_guess);
//       std::cout << "Normal Distribution Transform has converged:" << ndt.hasConverged()
//                 << ", score: " << ndt.getFitnessScore() << std::endl;
//       //输出到cloud3.pcd文件
//       pcl::transformPointCloud(*scan_ptr, *output_cloud, ndt.getFinalTransformation());
//       pcl::io::savePCDFileASCII("cloud3.pcd", *output_cloud);
//
//       // auto transform_matrix = ndt.getFinalTransformation();
//       // //输出转移矩阵
//       // std::cout << "transform_matrix: " << transform_matrix << '\n';
//       //可视化点云
//       visualizer(map_ptr, scan_ptr);
//
//     }
//     else
//       return;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_collection");
    ros::NodeHandle n;
    ROS_INFO("data collecting...");

    ros::Subscriber sub_odom = n.subscribe<nav_msgs::Odometry>("odom", 10, odomcallback);
    ros::Subscriber sub_imu = n.subscribe<sensor_msgs::Imu>("imu", 10, imucallback);
    // ros::Subscriber sub_rslidar = n.subscribe<sensor_msgs::PointCloud2>("rslidar_points", 100000, pc_callback);
    ros::Subscriber sub_gps = n.subscribe<sensor_msgs::NavSatFix>("gps_ori", 10, gpscallback);

    double x,y,z;
    double roll, pitch, yaw;

    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while (n.ok()) {

      tf::StampedTransform transform;
      try{
        listener.lookupTransform("/map_laser", "/laserbase_footprint",
                                ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
     }

     x = transform.getOrigin().x();
     y = transform.getOrigin().y();

     transform.getBasis().getEulerYPR(yaw, pitch, roll);

     ROS_INFO("car_x  : %.3f, car_y : %.3f", x, y);
     ROS_INFO("odom_x : %.3f,odom_y : %.3f", odom_x, odom_y);
     ROS_INFO("roll      : %.3f, pitch      : %.3f, yaw      : %.3f", roll, pitch, yaw);
     ROS_INFO("imu_roll  : %.3f, imu_pitch  : %.3f, imu_yaw  : %.3f", imu_roll, imu_pitch, imu_yaw);
     ROS_INFO("odom_roll : %.3f, odom_pitch : %.3f, odom_yaw : %.3f", odom_roll, odom_pitch, odom_yaw);

     #ifdef save_data_to_file
     ofstream outfile;
     outfile.open("data.txt", ios::binary | ios::app | ios::in | ios::out);
     if(!outfile) cout<<"error"<<endl;

     outfile<<"Timestamps:"<< data_time_stamps << "," <<"senser_type:"<< "map_pos" <<","<<"x:"<<x<<","<<"y:"<<y<<"\n";
     outfile<<"Timestamps:"<< data_time_stamps << "," <<"senser_type:"<< "odom_pos" <<","<<"x:"<<odom_x<<","<<"y:"<<odom_y<<"\n";
     outfile<<"Timestamps:"<< data_time_stamps << "," <<"senser_type:"<< "map_acc" <<","<<"roll:"<< roll <<","<< "pitch:" << pitch <<","<<"yaw:"<<yaw<<"\n";
     outfile<<"Timestamps:"<< data_time_stamps << "," <<"senser_type:"<< "imu_acc" <<","<<"roll:"<<imu_roll<<","<<"pitch:"<<imu_pitch<<","<<"yaw:"<<imu_yaw<<"\n";
     outfile<<"Timestamps:"<< data_time_stamps << "," <<"senser_type:"<< "odom_acc" <<","<<"roll:"<<odom_roll<<","<<"pitch:"<<odom_pitch<<","<<"yaw:"<<odom_yaw<<"\n";

     outfile.close();//关闭文件，保存文件。
     ROS_INFO("save data row : %d",data_time_stamps);
     #endif

     #ifdef save_data_to_file_GPS
	
     //ofstream outfile;
     outfile.open("data_GPS_TOPIC.txt", ios::binary | ios::app | ios::in | ios::out);

     outfile<<fixed<<setprecision(10)<<"senser_type:"<< "GPS_pos"<<","<<"la:"<<gps_la<<","<<"lo:"<<gps_lo<< ";\\" <<"\n";

     outfile.close();//关闭文件，保存文件。
     ROS_INFO("save data row : %d",data_time_stamps);
     #endif

     data_time_stamps++;

     ros::spinOnce();
     rate.sleep();
    }

    return 0;
}
