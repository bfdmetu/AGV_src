#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

float wheel_speed[2];


void velCallbacktest(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    float wheel_track = 0.6f ;    //两轮间距，单位是m
    float linear_temp=0,angular_temp=0;//暂存的线速度和角速度

    //string port("/dev/ttyUSB0");    //小车串口号
    //unsigned long baud = 115200;    //小车串口波特率
    //serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //配置串口

    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s

    //将转换好的小车速度分量为左右轮速度
    wheel_speed[0] = linear_temp - 0.5f*angular_temp*wheel_track ;  //m/s
    wheel_speed[1] = linear_temp + 0.5f*angular_temp*wheel_track ;  //m/s
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_get_test");
    ros::NodeHandle n;
    ros::Subscriber subvel = n.subscribe("cmd_vel", 20, velCallbacktest); //订阅/cmd_vel主题
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ROS_INFO("left wheel vel: %f, rightwheel vel: %f",wheel_speed[0],wheel_speed[1]);
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}
