/***
***
*** rplidar_client.cpp
*** milemeter_code
***
***/

#include "math.h"
#include "ros/ros.h"
#include "my_serial_port.h"
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


#define RAD2DEG(x) ((x)*180./M_PI)
#define MAXSIZE 1024
#define ADV_MSG_LENGTH 11

#define X_Offsite 2.0
#define Y_Offsite 3.0


class send_status
{
public:
    serial_port sp1;
    serial_port sp3;
    //unsigned char xpos_h;
    //unsigned char xpos_l;
    //unsigned char ypos_h;
    //unsigned char zpos_l;
    //unsigned char yaw_h;
    //unsigned char yaw_l;
    char barrier_f;
    char send_buff[ADV_MSG_LENGTH];
    char rcv_buff1[MAXSIZE];
    char rcv_buff_save1[MAXSIZE];

    char rcv_buff3[MAXSIZE];
    char rcv_buff_save3[MAXSIZE];

    char lamp_color;
    char lamp_time;
    int save_end1;

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
    float cur_odom_th;        //当前odom的角度

    char ADVstatus;
    char speedflag;
    char powerstate;

    int motor_flag;
    int TestOdemFlag;
    int last_odom_x;
    int last_odom_y;
    float cur_odom_th_z;

    fd_set rd1,rd3;
};


void lampCallback(const std_msgs::UInt8MultiArray::ConstPtr& data);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void velCallback(const geometry_msgs::Twist::ConstPtr & cmd_input);
void velCallback_motorspeed(const geometry_msgs::Twist::ConstPtr & cmd_input);
void poscallback(const geometry_msgs::PoseStamped::ConstPtr& mypose);
void sp1operation();
void sp3operation();
int calculate_odom(nav_msgs::Odometry & odom);

void initial_all();
char bcc_check(char *buf,int length);
char high_two(double f);
char low_two(double f);
float get_yaw(float ow,float ox,float oy,float oz);
char bcc_check(char *buf,int start,int len);
bool getOdomPose( double& x, double& y, double& yaw, tf::TransformListener &listener, tf::StampedTransform &transform );

send_status ss;


int main(int argc, char **argv)
{
    //定义节点名称
    ros::init(argc, argv, "rplidar_node_client");

    //初始化参数
    initial_all();

    ros::NodeHandle node;
    ros::Subscriber sub_0 = node.subscribe<sensor_msgs::LaserScan>("scan", 1000, scanCallback);    //订阅雷达主题，扫描障碍物
    ros::Subscriber sub_1 = node.subscribe<geometry_msgs::Twist>("/turtle1/cmd_vel", 20, velCallback); //订阅/turtle1/cmd_vel主题 测试使用
    ros::Subscriber sub_2 = node.subscribe<geometry_msgs::Twist>("cmd_vel", 20, velCallback_motorspeed); //订阅move_base发出的速度主题，将速度转换到电机进行运动

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

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Rate loop_rate(10);

    int cur_time=clock();
    int last_time=clock()-10;


    while (ros::ok())
    {
        cur_time=clock();
        //一百毫秒 主程序运行在10HZ
      	if((int(cur_time - last_time)) >= 100000)
        {
          //操作串口下发速度控制指令以及接收VCU的指令
          sp1operation();
	        last_time = cur_time;
          //利用从VCU得到的编码器数据进行里程计计算
          calculate_odom(odom);
          //发布里程计主题
          odom_pub.publish(odom);
        }
        ros::spinOnce();
    }

    return 0;
}

void initial_all()
{
    ss.save_end1=0;

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
    ss.cur_odom_th=0.0;        //当前odom的角度

    ss.ADVnum=0x01;
    ss.ADVstatus=0x41;
    ss.speedflag=0x00;
    ss.powerstate=0x00;
    ss.lamp_color=0x04;
    ss.lamp_time=0x20;
}

/*串口发送底盘驱动控制帧，并接收采集数据帧
 *输入：无
 *输出：无
 */
void sp1operation()
{
    //ROS_INFO("I heard a laser scan");
    int i=0;
    if(ss.location[0]<0) ss.barrier_f|=0x80;
    if(ss.location[1]<0) ss.barrier_f|=0x40;

    ss.send_buff[0]=0x10;
    ss.send_buff[1]=0x81;
    ss.send_buff[2]=high_two(ss.speed[0]);
    ss.send_buff[3]=low_two(ss.speed[0]);
    ss.send_buff[4]=high_two(ss.speed[1]);
    ss.send_buff[5]=low_two(ss.speed[1]);
    ss.send_buff[6]=ss.barrier_f;
    //ss.send_buff[7]=ss.lamp_color;
    ss.send_buff[7]=ss.motor_flag;
    ss.send_buff[8]=ss.lamp_time;
    //ss.send_buff[7]=0x04;//绿灯4
    //ss.send_buff[8]=0x10;//信号灯时间
    ss.send_buff[9]=bcc_check(ss.send_buff,9);
    ss.send_buff[10]=0x03;

    write(ss.sp1.return_port(), ss.send_buff, ADV_MSG_LENGTH);

    FD_ZERO(&ss.rd1);
    FD_SET(ss.sp1.return_port(),&ss.rd1);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    //fd method to read sp
    //while(FD_ISSET(ss.sp.return_port(),&ss.rd1))
    int retval=0;
    retval=select(ss.sp1.return_port()+1,&ss.rd1, NULL, NULL, &tv);
    if(retval < 0) perror("select error!\n");
    else
    {
        if(retval && FD_ISSET(ss.sp1.return_port(), &ss.rd1))
	      {
            int s1_recv_len=read(ss.sp1.return_port(), ss.rcv_buff1, MAXSIZE);
            for(i=0; i<s1_recv_len; i++ )
            {
                ss.rcv_buff_save1[ss.save_end1] = ss.rcv_buff1[i];
                ss.save_end1++;
            		if(ss.save_end1 >= MAXSIZE) ss.save_end1=0;
            }
            if(ss.save_end1 >= ADV_MSG_LENGTH)
            {
                for(i=0;i<=ss.save_end1-ADV_MSG_LENGTH;i++)
                {
                    if(ss.rcv_buff_save1[i]==0x10 && (unsigned char)ss.rcv_buff_save1[i+1]==0x82)
                    {
                       if(bcc_check(ss.rcv_buff_save1,i,ADV_MSG_LENGTH-2) == ss.rcv_buff_save1[i+ADV_MSG_LENGTH-2])
                        {

                    				ss.left_enc = int(ss.rcv_buff_save1[i+2]*0xff + ss.rcv_buff_save1[i+3]);
                    				ss.right_enc = int(ss.rcv_buff_save1[i+4]*0xff + ss.rcv_buff_save1[i+5]);

                            ss.save_end1=0;
                        }
                    }
                }
            }
	      }
    }
}



int motor_angle_limit(float limit_angle)
{
    float vehicle_attitude_angle = 0;
    float motor_angle = ss.cur_odom_th_z*-100.0;
    float motor_to_vehicle_angle = 0;
    int motor_angle_limit_flag = 0;// 0 no limit , 2 limit right turn , 3 limit left turn
    //calculate the motor_to_vehicle_angle "-" is represent left , "+" is represent right
    if(abs(motor_angle - vehicle_attitude_angle) > 180)
    {
       if ((motor_angle - vehicle_attitude_angle) >= 0)
       {
          motor_to_vehicle_angle = 360 - abs(motor_angle - vehicle_attitude_angle);
       }
       else
       {
          motor_to_vehicle_angle = -(360 - abs(motor_angle - vehicle_attitude_angle));
       }
    }
    else
    {
       motor_to_vehicle_angle = motor_angle - vehicle_attitude_angle;
    }

    if(motor_to_vehicle_angle > -limit_angle && motor_to_vehicle_angle < limit_angle)
    {
       motor_angle_limit_flag = 0;//no limit
    }
    else if(motor_to_vehicle_angle > limit_angle)
    {
       motor_angle_limit_flag = 2;//limit right turn
    }
    else if(motor_to_vehicle_angle < -limit_angle)
    {
       motor_angle_limit_flag = 3;//limit left turn
    }

    return motor_angle_limit_flag;
}


void velCallback_motorspeed(const geometry_msgs::Twist::ConstPtr & cmd_input)
{
    float wheel_track = 0.32f ;    //两轮间距，单位是m
    float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
    int Motor_angle_limit_flag = 0;//1 forbid left,2 forbit right

    angular_temp = cmd_input->angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = cmd_input->linear.x ;//获取/cmd_vel的线速度.m/s

    //将转换好的小车速度分量为左右轮速度
    ss.speed[0] = linear_temp - 0.5f*angular_temp*wheel_track ;  //m/s  左轮
    ss.speed[1] = linear_temp + 0.5f*angular_temp*wheel_track ;  //m/s  右轮

    ss.speed[0] = ss.speed[0]/0.3*0.1;
    ss.speed[1] = ss.speed[1]/0.3*0.1;

    if (ss.speed[0] > 0.1)
    {
        ss.speed[0] = 0.1;
    }
    if (ss.speed[1] > 0.1)
    {
        ss.speed[1] = 0.1;
    }
    if (ss.speed[0] < -0.1)
    {
        ss.speed[0] = -0.1;
    }
    if (ss.speed[1] < -0.1)
    {
        ss.speed[1] = -0.1;
    }

    //屏蔽旋转限制
    //Motor_angle_limit_flag = motor_angle_limit(40);//1 forbid left,2 forbit right

    if(ss.speed[0] > 0)
    {
      if(ss.speed[1] > 0)
        ss.motor_flag = 0;//两个电机都是向前
      else
        {
          if(Motor_angle_limit_flag == 2)
          {
            ROS_INFO(" right turn limit !! ");
            ss.speed[0] = 0;
            ss.speed[1] = 0;
          }
          ss.motor_flag = 2;//右转
        }
    }
    else
    {
      if(ss.speed[1] > 0)
        {
          if(Motor_angle_limit_flag == 3)
          {
            ROS_INFO(" left turn limit !! ");
            ss.speed[0] = 0;
            ss.speed[1] = 0;
          }
          ss.motor_flag = 3;//左转
        }
      else
        ss.motor_flag = 1;//后退
    }

}

void velCallback(const geometry_msgs::Twist::ConstPtr & cmd_input)//订阅/cmd_vel主题回调函数
{
    float wheel_track = 0.6f ;    //两轮间距，单位是m
    float linear_temp=0,angular_temp=0;//暂存的线速度和角速度

    angular_temp = cmd_input->angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = cmd_input->linear.x ;//获取/cmd_vel的线速度.m/s

    if(angular_temp == 2)
    {
        ROS_INFO(" turn left!! ");

      ss.lamp_time = 0x02+1;//左转
    }
    else if (angular_temp == -2)
    {
        ROS_INFO(" turn right!! ");

      ss.lamp_time = 0x03+1;//右转
    }
    if(linear_temp == 2)
    {
        ROS_INFO(" go forward!! ");

      ss.lamp_time = 0x00+1;//前进
      ss.TestOdemFlag = 1;
      ss.last_odom_x=ss.cur_odom_x;
      ss.last_odom_y=ss.cur_odom_y;

    }
    else if(linear_temp == -2)
    {
        ROS_INFO(" go backward!! ");

      ss.lamp_time = 0x01+1;//后退
      ss.TestOdemFlag = 0;

    }
}



void lampCallback(const std_msgs::UInt8MultiArray::ConstPtr& data)   //lamp color lamp time
{
    ss.lamp_color=data->data[0];
    ss.lamp_time=data->data[1];
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    float range=0.0;
    int bflag=0;
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
        range=scan->ranges[i];
        if(-180<degree<-120||120<degree<180)
    	{
    	    if(range<0.7)
                {
                    // ROS_INFO("degree : [%f] , range : [%f]", degree, range);
    	        bflag++;
                }
    	}
    }
    if(bflag>5) ss.barrier_f=1;
    else ss.barrier_f=0;
    /**************test****************************/
    //ss.send_buff[0]=0x10;
    //ss.send_buff[1]=0x81;

    //write(ss.sp.return_port(), ss.send_buff, 2);
    /******************test************************/
}

void poscallback(const geometry_msgs::PoseStamped::ConstPtr& mypose)
{
    float ow,ox,oy,oz;
    ss.location[0]=mypose->pose.position.x+X_Offsite;
    ss.location[1]=mypose->pose.position.y+Y_Offsite;
    ox=mypose->pose.orientation.x;
    oy=mypose->pose.orientation.y;
    oz=mypose->pose.orientation.z;
    ow=mypose->pose.orientation.w;
    ss.location[2]=get_yaw(ow,ox,oy,oz);

    //ROS_INFO("x pos : %f , y pos : %f , z angle: %f", xpos , ypos , yaw);
    //ssoperation();
}

char bcc_check(char *buf,int length)
{
    unsigned char checksum=0;
    for(int i=0 ; i<length ; i++ ) checksum^=buf[i];
    return checksum;
}

char high_two(double f)
{
    int temp=(int)(fabs(f)*10000);
    return (char)(temp >> 8);
}
char low_two(double f)
{
    int temp=(int)(fabs(f)*10000);
    return (char)(temp);
}

float get_yaw(float ow,float ox,float oy,float oz)
{
    float yaw=atan2f(2*(ow*oz+ox*oy),1-2*(oy*oy+oz*oz));
    yaw=RAD2DEG(yaw)+180;
    return yaw;
}
char bcc_check(char *buf,int start,int len)
{

    char checksum=0;
    for(int i=start; i<start+len; i++) checksum^=buf[i];
    return checksum;
}


bool getOdomPose( double& x, double& y, double& yaw, tf::TransformListener &listener, tf::StampedTransform &transform )
{
    try
    {
        listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    x = transform.getOrigin().x()+X_Offsite;
    y = transform.getOrigin().y()+Y_Offsite;
    double pitch,roll;
    transform.getBasis().getEulerYPR(yaw, pitch, roll);
    yaw=RAD2DEG(yaw)+180;

    return true;
}

int calculate_odom(nav_msgs::Odometry & odom)
{
    geometry_msgs::Quaternion odom_quat; //四元数变量
    static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
    geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息

    int current_time=clock();
    float ticks_per_meter=3670.86509;   //设定多少个脉冲前进一米
    float wheel_track=1.555;          //轮间距
    float dt=float(float(current_time-ss.last_time)/CLOCKS_PER_SEC);   //与上次接受odom的时间差

	if (ss.left_enc > 65536/2) {
		ss.left_enc -= 65536;
	}
	if (ss.right_enc > 65536/2) {
		ss.right_enc -= 65536;
	}

    float dleft=float(ss.left_enc)/ticks_per_meter;   //与上次接受odom的左轮移动差
    float dright=float(ss.right_enc)/ticks_per_meter;//与上次接受odom的右轮移动差
    //ss.last_left_enc=ss.left_enc;
    //ss.last_right_enc=ss.right_enc;

    if(ss.motor_flag == 0)
    {
      //前进说明 dleft dright > 0
    }
    else if(ss.motor_flag == 1)
    {
      //后退 dleft dright < 0
      dleft = -dleft;
      dright = -dright;
    }
    else if(ss.motor_flag == 2)
    {
      //右转 dright < 0
      dright = -dright;
    }
    else if(ss.motor_flag == 3)
    {
      //左转 dleft < 0
      dleft = -dleft;
    }


    float dxy_ave = (dright + dleft) / 2.0;
    float dth = (dright - dleft) / wheel_track;
    float vxy = dxy_ave / dt;
    float vth = dth / dt;

    if (dxy_ave != 0.0)
    {
        float dx = cos(dth) * dxy_ave;
        float dy = -sin(dth) * dxy_ave;
        ss.cur_odom_x += (cos(ss.cur_odom_th) * dx - sin(ss.cur_odom_th) * dy);
        ss.cur_odom_y += (sin(ss.cur_odom_th) * dx + cos(ss.cur_odom_th) * dy);
    }

    if (dth != 0.0)
        ss.cur_odom_th += dth;

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
    odom.twist.twist.linear.x = vxy;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vth;

    ss.last_time=current_time;
    return 0;
}
