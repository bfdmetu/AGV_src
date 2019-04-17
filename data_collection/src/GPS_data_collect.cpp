//
//  GPS_data_collect.cpp
//  GPS_data_collect
//
//  Created by 钟海兴 on 2019/03/11.
//  Copyright © 2019 钟海兴. All rights reserved.
//

#include "GPS_data_collect.hpp"
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


#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/NavSatFix.h>

#include <fstream>
using namespace std;

#include <sensor_msgs/PointCloud2.h>

#define CONTROL_MOTOR_MSG_LENGTH 10
#define MAXSIZE 1024
#define ADV_MSG_LENGTH 12

#define save_data_to_file

using std::string;

class send_status
{
public:
    serial_port sp1;
    serial_port sp3;

    char rcv_buff1[MAXSIZE];
    char rcv_buff_save1[MAXSIZE];

    char rcv_buff3[MAXSIZE];
    char rcv_buff_save3[MAXSIZE];

    int save_end1;
    int save_end3;

    fd_set rd1,rd3;

    double timestamps_GPS_data;
    double latitude_data, longitude_data;

};



send_status ss;

//初始化函数
void initial_all();

//操作gps串口 下发控制数据帧以及收取采集数据帧
void gps_sp_operation();
//发布GPS主题数据
int gps_data_input(sensor_msgs::NavSatFix & gps_ori);

//接收的数据帧数
int32_t data_num = 0;
//接收数据帧时间点
int cur_time=clock();
int last_time=clock()-10;
//从缓冲区读取的数据长度
int s1_recv_len_all = 0;





int main(int argc, char **argv)
{
    //定义节点名称
    ros::init(argc, argv, "GPS_data_collect");
    //初始化参数
    initial_all();
    ROS_INFO("gps data collecting...");

    ros::NodeHandle gps_node;

    ros::Publisher gps_pub = gps_node.advertise<sensor_msgs::NavSatFix>("gps_ori", 10); //定义要发布/gps_ori主题

    sensor_msgs::NavSatFix gps_ori;//定义GPS主题对象
/*
    float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
                            0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
                            0,  0,    99999, 0,     0,     0,  // covariance on gps_z
                            0,  0,    0,     99999, 0,     0,  // large covariance on rot x
                            0,  0,    0,     0,     99999, 0,  // large covariance on rot y
                            0,  0,    0,     0,     0,     0.01};  // large covariance on rot z
    //载入covariance矩阵
    for(int i = 0; i < 36; i++)
    {
        gps_ori.position_covariance[i] = covariance[i];
    }

*/

    ros::Rate rate(10.0);

    cur_time=clock();
    last_time=clock()-10;

    while (gps_node.ok()) {

      // ROS_INFO("gps data collecting...");
      //采集GPS串口数据
      gps_sp_operation();
      //处理GPS数据到主题
      gps_data_input(gps_ori);
      //发布GPS主题
      gps_pub.publish(gps_ori);
      ros::spinOnce();
      rate.sleep();

    }

    return 0;

}

/*初始化node初始参数
 *无输入
 *无输出
 */
void initial_all()
{

  ss.sp1.open_port(1);
  ss.sp1.set_port();

}



/*串口发送gps控制帧，并接收采集数据帧
 *输入：无
 *输出：无
 */
void gps_sp_operation()
{
    /***send data***/

    // //ROS_INFO("I heard a laser scan");
    // ss.Send_buff[0] = 0xff;                     //帧头
    // ss.Send_buff[1] = 0xfe;                     //帧头
    // ss.Send_buff[2] = ss.control_motor_mode;    //速度控制模式
    // ss.Send_buff[3] = ss.speed_x/256;           //x轴速度高八位
    // ss.Send_buff[4] = ss.speed_x%256;           //x轴速度低八位
    // ss.Send_buff[5] = ss.speed_y/256;           //y轴速度高八位
    // ss.Send_buff[6] = ss.speed_y%256;           //y轴速度低八位
    // ss.Send_buff[7] = ss.speed_z/256;           //z轴速度高八位
    // ss.Send_buff[8] = ss.speed_z%256;           //z轴速度低八位
    // ss.Send_buff[9] = ss.dir_speed;             //各轴速度方向
    //
    // //发送控制数据帧
    // write(ss.sp1.return_port(), ss.Send_buff, CONTROL_MOTOR_MSG_LENGTH);


    /***read data***/
    string temp_text;
    const char *ch;
    char a[512] = {0};

    string timestamps_GPS;
    string latitude,latitude_n_s;
    string longitude,longitude_e_w;

    //发送完成后读取接收数据帧
    FD_ZERO(&ss.rd1);
    FD_SET(ss.sp1.return_port(),&ss.rd1);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    //fd method to read sp
    //while(FD_ISSET(ss.sp.return_port(),&ss.rd1))

    int retval=0;
    retval=select(ss.sp1.return_port()+1,&ss.rd1,NULL,NULL,NULL);
    if(retval < 0)
        perror("select error!\n");
    else
    {
        if(retval && FD_ISSET(ss.sp1.return_port(),&ss.rd1))
        {
            // last_time=clock()-10;
            // cur_time = clock();
            // int s1_recv_len = 0;
            // int s1_recv_len_1 = 0;
            // s1_recv_len_1 = read(ss.sp1.return_port(),ss.rcv_buff1,MAXSIZE);
            // while ( (cur_time - last_time < 50000)) {
            //   cur_time = clock();
            //   s1_recv_len += s1_recv_len_1;
            //   s1_recv_len_1 = read(ss.sp1.return_port(),ss.rcv_buff1,MAXSIZE);
            // }
            cur_time = clock();
            int sub_value = cur_time - last_time;
            //本来想着如果上下两帧数据时间相差不大就把两帧数据合并，但是合并时会产生丢失的情况
            if (cur_time - last_time > 100) {

              // ROS_INFO("here is receive cur time: %d , here is the sub value : %d ", cur_time, sub_value);
              // ROS_INFO("here is receive data num: %d ", data_num);
              // ROS_INFO("here is receive data lenth: %d ", s1_recv_len_all);
              // ROS_INFO("here is receive data : %s ", ss.rcv_buff_save1);

              //定位"$GNRMC"帧头
              if (strstr(ss.rcv_buff_save1, "$GNRMC") != NULL) {
                ROS_INFO("processing data ...");
                //定位"$GNRMC"帧头
                ch = strstr(ss.rcv_buff_save1, "$GNRMC");
                //跳过"$GNRMC,"
                ch += sizeof("$GNRMC");
                //复制字符串
                strcpy(a, ch);
                temp_text = a;
                int senser_type_loca = temp_text.find(",");
                string output_GPS_temp =  temp_text.substr(0, senser_type_loca);
                std::cout << "here is timestamps_GPS : " << output_GPS_temp << std::endl;
                timestamps_GPS = output_GPS_temp;

                //跳过",hhmmss.ss,A"
                ch += sizeof(",A") + output_GPS_temp.length();
                //复制字符串
                strcpy(a, ch);
                temp_text = a;
                senser_type_loca = temp_text.find(",");
                output_GPS_temp =  temp_text.substr(0, senser_type_loca);
                std::cout << "here is latitude : " << output_GPS_temp << std::endl;
                latitude = output_GPS_temp;
		            ss.latitude_data = atof(latitude.c_str());

                //跳过纬度数据
                ch += (sizeof(',') + output_GPS_temp.length());
                //复制字符串
                strcpy(a, ch);
                temp_text = a;
                senser_type_loca = temp_text.find(",");
                output_GPS_temp =  temp_text.substr(0, senser_type_loca);
                std::cout << "here is latitude N or S : " << output_GPS_temp << std::endl;
                latitude_n_s = output_GPS_temp;

                //跳过南北纬
                ch += (sizeof(',') + output_GPS_temp.length());
                //复制字符串
                strcpy(a, ch);
                temp_text = a;
                senser_type_loca = temp_text.find(",");
                output_GPS_temp =  temp_text.substr(0, senser_type_loca);
                std::cout << "here is longitude : " << output_GPS_temp << std::endl;
                longitude = output_GPS_temp;
		            ss.longitude_data = atof(longitude.c_str());

                //跳过经度数据
                ch += (sizeof(',') + output_GPS_temp.length());
                //复制字符串
                strcpy(a, ch);
                temp_text = a;
                senser_type_loca = temp_text.find(",");
                output_GPS_temp =  temp_text.substr(0, senser_type_loca);
                std::cout << "here is longitude E or W : " << output_GPS_temp << std::endl;
                longitude_e_w = output_GPS_temp;

                #ifdef save_data_to_file
                ofstream outfile;
                outfile.open("gps_data.txt", ios::binary | ios::app | ios::in | ios::out);
                if(!outfile) cout<<"error"<<endl;

                outfile<<"time:"<< timestamps_GPS << "," <<"la:"<< latitude <<","<<"lo:"<< longitude << ";\\" <<"\n";


                outfile.close();//关闭文件，保存文件。
                #endif

              }


              //重新开始计数
              ss.save_end1=0;
              s1_recv_len_all = 0;
              data_num++;
            }
            else {
              // ss.save_end1--;
            }
            //现在时刻保存为上一时刻
            last_time = cur_time;

            //从串口缓冲区中读取数据长度
            int s1_recv_len = 0;
            char rec_buff_gps[MAXSIZE];

            for (size_t loop_i = 0; loop_i < 1; loop_i++) {
              /* code */
              s1_recv_len = read(ss.sp1.return_port(),rec_buff_gps,MAXSIZE);
              s1_recv_len_all += s1_recv_len;
              //从串口缓冲区中读取数据
              for(int i=0; i<s1_recv_len; i++ ){
                  ss.rcv_buff_save1[ss.save_end1]=rec_buff_gps[i];
                  ss.save_end1++;
                  if(ss.save_end1>=MAXSIZE)
                      ss.save_end1=0;
              }
              ss.rcv_buff_save1[ss.save_end1] = '\0';
            }

            tcflush(ss.sp1.return_port(),TCIFLUSH);



        }
    }

   
}

/*GPS数据输入主题与发布
 *输入：GPS主题
 *输出：无
 */
int gps_data_input(sensor_msgs::NavSatFix & gps_ori) {

   //载入坐标（tf）变换时间戳
    gps_ori.header.stamp = ros::Time::now();
    //GPS坐标系
    gps_ori.header.frame_id = "gps_ori";
    //GPS经纬度数据
    int la = int(ss.latitude_data/100);
    gps_ori.latitude = la + (ss.latitude_data-la*100)/60.0;
    int lo = int(ss.longitude_data/100);
    gps_ori.longitude = lo + (ss.longitude_data-lo*100)/60.0;
    // gps_ori.latitude = ss.latitude_data;
    // gps_ori.longitude = ss.longitude_data;
    //海拔高度
    gps_ori.altitude = 10;
    //协方差类型
    gps_ori.position_covariance_type = 0;

}
