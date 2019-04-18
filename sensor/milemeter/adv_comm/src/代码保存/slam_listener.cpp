#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <visualization_msgs/MarkerArray.h>
void poscallback(const geometry_msgs::PoseStamped::ConstPtr& mypose)
{
    ROS_INFO("x pos : %f , y pos : %f , z angle: %f",mypose->pose.position.x , mypose->pose.position.y , mypose->pose.orientation.z);
    //ROS_INFO("x pos :  , y pos : );
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_listener");
    ros::NodeHandle n;
    ROS_INFO("get position ");

    ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("slam_out_pose", 10, poscallback);

    ros::spin();

    return 0;
}
