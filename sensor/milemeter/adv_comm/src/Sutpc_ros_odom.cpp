//
//  Sutpc_ros_odom.cpp
//  sutpc_ros_odom
//
//  Created by 钟海兴 on 2018/12/29.
//  Copyright © 2018 钟海兴. All rights reserved.
//

#include "Sutpc_ros_odom.hpp"
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

#define CONTROL_MOTOR_MSG_LENGTH 10
#define MAXSIZE 1024
#define ADV_MSG_LENGTH 12


#define RAD2DEG(x) ((x)*180./M_PI)

#define X_Offsite 2.0
#define Y_Offsite 3.0

class send_status
{
public:
    serial_port sp1;
    serial_port sp3;

    char rcv_buff1[MAXSIZE];
    unsigned char rcv_buff_save1[MAXSIZE];

    char rcv_buff3[MAXSIZE];
    char rcv_buff_save3[MAXSIZE];

    char lamp_color;
    char lamp_time;
    int save_end1;
    int save_end3;
    double location[3];
    double speed[2];
    char ADVnum;

    int left_enc;
    int right_enc;
    int last_left_enc;
    int last_right_enc;
    int last_time;
    float cur_odom_x;         //当前odom的x坐标
    float cur_odom_y;         //当前odom的y坐标
    double cur_odom_th;        //当前odom的角度

    char ADVstatus;
    char speedflag;
    char powerstate;

    int motor_flag;
    int TestOdemFlag;
    int last_odom_x;
    int last_odom_y;
    float cur_odom_th_z;


    //*****小车参数*****//

    //小车轮子直径
    float diameter;
    //底盘长、宽;
    float chassis_a,chassis_b;

    //*****下发数据*****//

    //串口下发的速度给定量
    unsigned int speed_x;
    unsigned int speed_y;
    unsigned int speed_z;
    //串口下发的速度方向
    char dir_speed;
    //串口下发数据帧
    unsigned char Send_buff[CONTROL_MOTOR_MSG_LENGTH];
    //速度控制模式
    char control_motor_mode;

    //*****采集数据*****//

    //各电机的速度大小
    int16_t motor_speed_A_encoder;
    int16_t motor_speed_B_encoder;
    int16_t motor_speed_C_encoder;
    int16_t motor_speed_D_encoder;
    //各电机的速度方向 只有三种状态 1代表速度数据为正，0代表停止，2代表速度数据为负
    char motor_speed_A_dir;
    char motor_speed_B_dir;
    char motor_speed_C_dir;
    char motor_speed_D_dir;
    //Z轴陀螺仪数据
    int32_t Z_gyro_speed;


    fd_set rd1,rd3;
};

send_status ss;

//初始化函数
void initial_all();
//订阅速度主题后，驱动底盘运动
void velCallback_motorspeed(const geometry_msgs::Twist::ConstPtr & cmd_input);
void velCallback_motorspeed_test(const geometry_msgs::Twist::ConstPtr & cmd_input);
//操作串口1 下发控制数据帧以及收取采集数据帧
void sp1operation();
//里程计计算函数
int calculate_odom(nav_msgs::Odometry & odom);

//*****工具函数*****//
int16_t get_motor_speed(u_char speed, u_char dir);

int last_time_1;

int main(int argc, char **argv)
{
    //定义节点名称
    ros::init(argc, argv, "sutpc_odom_node");
    //初始化参数
    initial_all();

    ros::NodeHandle node;
    //订阅速度主题
   ros::Subscriber sub2 = node.subscribe<geometry_msgs::Twist>("cmd_vel", 20, velCallback_motorspeed);
    // ros::Subscriber sub2 = node.subscribe<geometry_msgs::Twist>("cmd_vel", 20, velCallback_motorspeed_test);
    //发布里程计主题
    ros::Publisher odom_pub= node.advertise<nav_msgs::Odometry>("odom", 10); //定义要发布/odom主题
    nav_msgs::Odometry odom;//定义里程计对象
    float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
      0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
      0,  0,    99999, 0,     0,     0,  // covariance on gps_z
      0,  0,    0,     99999, 0,     0,  // large covariance on rot x
      0,  0,    0,     0,     99999, 0,  // large covariance on rot y
      0,  0,    0,     0,     0,     0.01};  // large covariance on rot z
    //载入covariance矩阵
    for(int i = 0; i < 36; i++)
    {
      odom.pose.covariance[i] = covariance[i];;
    }

    ros::Rate loop_rate(10);
    int cur_time=clock();
    int last_time=clock()-10;
    while (ros::ok())
    {
       cur_time=clock();
       //间隔一百毫秒下发串口数据，数据下发频率为20HZ
       if((int(cur_time - last_time)) >= 100000/2)
       {
           last_time = cur_time;
           //发送控制数据帧并接收采集数据帧
           sp1operation();
          //  //测试Z轴转换系数
          //  if ((int(cur_time - last_time_1)) >= 3000000) {
          //    ss.speed_z = 0;
          //  }
           //计算里程计
           calculate_odom(odom);
           //发布里程计主题
           odom_pub.publish(odom);
       }
        ros::spinOnce();
    }

    return 0;

}

/*初始化odom初始参数
 *无输入
 *无输出
 */
void initial_all()
{
  ss.save_end1=0;
  ss.save_end3=0;
  for(int i=0;i<3;i++)
    ss.location[i]=0.0;
  for(int i=0;i<2;i++)
    ss.speed[i]=0.0;
  ss.sp1.open_port(11);
  ss.sp1.set_port();

  ss.last_time=clock();
  ss.last_left_enc=0;
  ss.last_right_enc=0;
  ss.cur_odom_x=0.0;         //当前odom的x坐标
  ss.cur_odom_y=0.0;         //当前odom的y坐标
  ss.cur_odom_th = -90.0*M_PI/180.0;        //当前odom的角度

  ss.ADVnum=0x01;
  ss.ADVstatus=0x41;
  ss.speedflag=0x00;
  ss.powerstate=0x00;
  ss.lamp_color=0x04;
  ss.lamp_time=0x20;


    //小车轮子直径设定
    ss.diameter = 0.1;
    ss.chassis_a = 0.503/2;
    ss.chassis_b = 0.452/2;
    //速度控制模式1，x,y,z轴速度给定模式
    ss.control_motor_mode = 0x01;
//    //速度控制模式2，对每个电机进行单独速度闭环控制
//    ss.control_motor_mode = 0x02;


}


/*订阅速度主题，每接收到一次速度就执行一次
 *输入：速度主题
 *输出：发送串口相应数据帧,测试Z轴转换系数
 */
void velCallback_motorspeed_test(const geometry_msgs::Twist::ConstPtr & cmd_input)
{

    if (cmd_input->linear.x < -0.4) {
        ss.speed_z = 20;
    }
    else if (cmd_input->linear.x > 0.4){
        ss.speed_z = 50;
    }
    if (cmd_input->angular.z < -0.4) {
        ss.speed_z = 70;
    }
    else if (cmd_input->angular.z > 0.4) {
        ss.speed_z = 120;
    }
    if (cmd_input->linear.x > -0.2 && cmd_input->linear.x <0.2){
      if (cmd_input->angular.z > -0.2 && cmd_input->angular.z < 0.2)
        ss.speed_z = 0;
    }
    ss.speed_x = 0;
    ss.speed_y = 0;

    //z轴方向,true为正
    bool speed_z_dir = true;

    if (speed_z_dir == true) {
        ss.dir_speed |= 0x01;
    }
    else{
        ss.dir_speed &= 0xfe;
    }
    last_time_1=clock();

}

/*订阅速度主题后，修改下发的全局速度参数
 *输入：速度主题
 *输出：无
 */
void velCallback_motorspeed(const geometry_msgs::Twist::ConstPtr & cmd_input)
{
    //下发底盘驱动的各轴速度给定量
    unsigned int speed_x = 0;
    unsigned int speed_y = 0;
    unsigned int speed_z = 0;
    //各轴速度方向,低三位分别代表各轴速度方向
    unsigned char dir_speed_temp;

    //判断各轴移动方向
    if (cmd_input->linear.x > 0) {
        dir_speed_temp |= 0x01<<1;
    }
    else{
        dir_speed_temp &= 0xfd;
    }
    if (cmd_input->linear.y < 0) {
        dir_speed_temp |= 0x01<<2;
    }
    else{
        dir_speed_temp &= 0xfb;
    }
    if (cmd_input->angular.z < 0) {
        dir_speed_temp |= 0x01;
    }
    else{
        dir_speed_temp &= 0xfe;
    }

   float angle = 90*M_PI/180;

   double speed_x_in = cmd_input->linear.x*cos(angle) - cmd_input->linear.y*sin(angle);
   double speed_y_in = cmd_input->linear.y*cos(angle) + cmd_input->linear.x*sin(angle);

    //x,y轴速度为 V = n*100/2700 * (pi*d)  （此处n为10ms移动脉冲）
    speed_x = fabs(speed_x_in)/(M_PI*ss.diameter)*27.0;
    speed_y = fabs(speed_y_in)/(M_PI*ss.diameter)*27.0;
    //0.0127465为z轴转换系数
    speed_z = fabs(cmd_input->angular.z)/0.0127465;


    //给定到下发串口数据
    ss.dir_speed = dir_speed_temp;
    ss.speed_x = speed_x;
    ss.speed_y = speed_y;
    ss.speed_z = speed_z;

}

/*串口发送底盘驱动控制帧，并接收采集数据帧
 *输入：无
 *输出：无
 */
void sp1operation()
{
    //ROS_INFO("I heard a laser scan");
    ss.Send_buff[0] = 0xff;                     //帧头
    ss.Send_buff[1] = 0xfe;                     //帧头
    ss.Send_buff[2] = ss.control_motor_mode;    //速度控制模式
    ss.Send_buff[3] = ss.speed_x/256;           //x轴速度高八位
    ss.Send_buff[4] = ss.speed_x%256;           //x轴速度低八位
    ss.Send_buff[5] = ss.speed_y/256;           //y轴速度高八位
    ss.Send_buff[6] = ss.speed_y%256;           //y轴速度低八位
    ss.Send_buff[7] = ss.speed_z/256;           //z轴速度高八位
    ss.Send_buff[8] = ss.speed_z%256;           //z轴速度低八位
    ss.Send_buff[9] = ss.dir_speed;             //各轴速度方向


    //发送控制数据帧
    write(ss.sp1.return_port(), ss.Send_buff, CONTROL_MOTOR_MSG_LENGTH);
    //发送完成后读取接收数据帧
    FD_ZERO(&ss.rd1);
    FD_SET(ss.sp1.return_port(),&ss.rd1);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    //fd method to read sp
    //while(FD_ISSET(ss.sp.return_port(),&ss.rd1))
    int retval=0;
    retval=select(ss.sp1.return_port()+1,&ss.rd1,NULL,NULL,&tv);
    if(retval < 0)
        perror("select error!\n");
    else
    {
        if(retval && FD_ISSET(ss.sp1.return_port(),&ss.rd1))
        {
            //从串口缓冲区中读取数据长度
            int s1_recv_len=read(ss.sp1.return_port(),ss.rcv_buff1,MAXSIZE);
            //从串口缓冲区中读取数据
            for(int i=0; i<s1_recv_len; i++ ){
                ss.rcv_buff_save1[ss.save_end1]=ss.rcv_buff1[i];
                ss.save_end1++;
                if(ss.save_end1>=MAXSIZE)
                    ss.save_end1=0;
            }
            //检测采集数据帧是否达到标准数据长度，根据开发手册，返回数据帧为12字节
            if(ss.save_end1>=ADV_MSG_LENGTH)
            {
                //
                for(int i=0;i<=ss.save_end1-ADV_MSG_LENGTH;i++)
                {
                    //检测帧头
                    if((unsigned char)ss.rcv_buff_save1[i]==0xff&&(unsigned char)ss.rcv_buff_save1[i+1]==0xfe)
                    {
                        //采集数据帧无bcc校验，直接读取数据
                        ss.motor_speed_A_encoder = get_motor_speed(ss.rcv_buff_save1[i+2], ss.rcv_buff_save1[i+3]);
                        ss.motor_speed_B_encoder = get_motor_speed(ss.rcv_buff_save1[i+4], ss.rcv_buff_save1[i+5]);
                        ss.motor_speed_C_encoder = get_motor_speed(ss.rcv_buff_save1[i+6], ss.rcv_buff_save1[i+7]);
                        ss.motor_speed_D_encoder = get_motor_speed(ss.rcv_buff_save1[i+8], ss.rcv_buff_save1[i+9]);
                        ss.Z_gyro_speed = ss.rcv_buff_save1[i+10]*256 + ss.rcv_buff_save1[i+11] - 32768;
                        //测试读取结果
                        // ROS_INFO("here are motors' status, A:%d  B:%d  C:%d  D:%d.", ss.motor_speed_A_encoder, ss.motor_speed_B_encoder, ss.motor_speed_C_encoder, ss.motor_speed_D_encoder);
                        // ROS_INFO("here is the z_speed : %d ", ss.Z_gyro_speed);

                        ss.save_end1=0;
                    }
                }
            }
        }
    }
}

/*工具函数：得到各电机的速度，带方向
 *输入：不带方向的速度
 *输出：带方向的速度
 */
int16_t get_motor_speed(u_char speed, u_char dir){
    int16_t output_speed;
    //各电机的速度方向 只有三种状态 0代表速度数据为正，1代表停止，2代表速度数据为负
    switch (dir) {
        case 0:
            output_speed = speed;
            break;

        case 1:
            output_speed = 0;
            break;

        case 2:
            output_speed = -speed;
            break;

        default:
            break;
    }

    return output_speed;
}

/*计算里程计函数
 *输入：里程计主题
 *输出：里程计主题
 */
int calculate_odom(nav_msgs::Odometry & odom)
{
    geometry_msgs::Quaternion odom_quat; //四元数变量
    static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
    geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息

    int current_time=clock();
    float dt=float(float(current_time-ss.last_time)/CLOCKS_PER_SEC);   //与上次接受odom的时间差
    float dx,dy;

    //定义各电机的速度
    float velocity_A,velocity_B,velocity_C,velocity_D;
    float velocity_dir_X,velocity_dir_Y,velocity_dir_w;

    //利用各个轮子编码器的值计算出各个轮子当前速度
    velocity_A = (ss.motor_speed_A_encoder * 100.0)/2700.0*M_PI*ss.diameter;
    velocity_B = (ss.motor_speed_B_encoder * 100.0)/2700.0*M_PI*ss.diameter;
    velocity_C = (ss.motor_speed_C_encoder * 100.0)/2700.0*M_PI*ss.diameter;
    velocity_D = (ss.motor_speed_D_encoder * 100.0)/2700.0*M_PI*ss.diameter;

    velocity_dir_X = ((velocity_B + velocity_D) - (velocity_A + velocity_C))/4;
    velocity_dir_Y = (velocity_A + velocity_B + velocity_C + velocity_D)/4;
    velocity_dir_w = ((velocity_C + velocity_D) - (velocity_A + velocity_B))/(4*(ss.chassis_a + ss.chassis_b));

    float angle = -90*M_PI/180;

    double velocity_dir_X_rot = velocity_dir_X*cos(angle) - velocity_dir_Y*sin(angle);
    double velocity_dir_Y_rot = velocity_dir_Y*cos(angle) + velocity_dir_X*sin(angle);
   

    //速度修正系数，经过测量，与实际距离都偏小了
    //乘上负号为了调整方向
    velocity_dir_X = velocity_dir_X_rot * -1.07;
    velocity_dir_Y = velocity_dir_Y_rot * -1.07;
    velocity_dir_w *= 1.22;

    dx = velocity_dir_X*dt;
    dy = velocity_dir_Y*dt;
    ss.cur_odom_x += (cos(ss.cur_odom_th) * dx - sin(ss.cur_odom_th) * dy);
    ss.cur_odom_y += (sin(ss.cur_odom_th) * dx + cos(ss.cur_odom_th) * dy);
    // ss.cur_odom_th += ((double)velocity_dir_w*(double)dt)%(M_PI*2.0);
    ss.cur_odom_th += velocity_dir_w*dt;
    while ( ss.cur_odom_th > M_PI*2.0  ) {
      ss.cur_odom_th -= M_PI*2.0;
    }
    // while ( ss.cur_odom_th < -M_PI ) {
    //   ss.cur_odom_th += M_PI*2.0;
    // }

    odom_quat = tf::createQuaternionMsgFromYaw(ss.cur_odom_th);//将偏航角转换成四元数
    ss.cur_odom_th_z = odom_quat.z;

    //载入坐标（tf）变换时间戳
    odom_trans.header.stamp = ros::Time::now();
    //发布坐标变换的父子坐标系
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint_wheel";
    //tf位置数据：x,y,z,方向
    odom_trans.transform.translation.x = ss.cur_odom_x;
    odom_trans.transform.translation.y = ss.cur_odom_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //发布tf坐标变化
    odom_broadcaster.sendTransform(odom_trans);
    //载入里程计时间戳
    odom.header.stamp = ros::Time::now();
    //里程计的父子坐标系
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint_wheel";
    //里程计位置数据：x,y,z,方向
    odom.pose.pose.position.x = ss.cur_odom_x;
    odom.pose.pose.position.y = ss.cur_odom_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //载入线速度和角速度
    odom.twist.twist.linear.x = velocity_dir_X;
    odom.twist.twist.linear.y = velocity_dir_Y;
    odom.twist.twist.angular.z = velocity_dir_w;

    ss.last_time=current_time;
    return 0;
}
