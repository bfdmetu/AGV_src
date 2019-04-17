//
//  GPS_data_collect.hpp
//  GPS_data_collect
//
//  Created by 钟海兴 on 2019/03/11.
//  Copyright © 2019 钟海兴. All rights reserved.
//

#ifndef GPS_data_collect_hpp
#define GPS_data_collect_hpp

#include     <stdio.h>
#include     <stdlib.h>
#include     <unistd.h>
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>
#include     <termios.h>
#include     <errno.h>

class serial_port
{
private:
    int port_fd;
public:
    void open_port(int i);
    void set_port();
    int return_port();
};

void serial_port::open_port(int i)
{
    if(i==1)
    {
      port_fd = open("/dev/ttyUSB009", O_RDWR | O_NOCTTY | O_NDELAY);
      printf("opening /dev/ttyUSB009 port!\n");
    }


    if (port_fd == -1)
    {
        /*
         * Could not open the port.
         */
        perror("open_port: Unable to open serial_port !!!! please check the serial_port!!");
    }
    else
    {
        fcntl(port_fd, F_SETFL, 0);
    }
}
void serial_port::set_port()
{
    struct  termios Opt;

    if  ( tcgetattr( port_fd , &Opt)  !=  0 ) {
        perror("SetupSerial 1 :");
    }

    // Opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    // Opt.c_oflag &= ~OPOST;
    //
    // Opt.c_cflag |= CLOCAL | CREAD;
    //
    // Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);


    /*//8bit数据位
     Opt.c_cflag &= ~CSIZE;
     Opt.c_cflag |= CS8;
     //清除校验位
     Opt.c_cflag &= ~PARENB;
     //1bit停止位
     Opt.c_cflag &= ~CSTOPB;*/




 Opt.c_cflag|=(CLOCAL|CREAD);//enable date receiver
 Opt.c_cflag&=~PARENB;//没有校验
 Opt.c_cflag&=~CRTSCTS;//没有数据流
 Opt.c_cflag&=~CSTOPB;//关闭两位停止位，就是一位停止位
 Opt.c_cflag&=~CSIZE;//设置数据位宽时打开掩码
 Opt.c_cflag|=CS8;//8位数据位


 //关闭ICRNL IXON 防止0x0d 0x11 0x13的过滤
 Opt.c_iflag&=~(IXON|IXOFF|IXANY);
 Opt.c_iflag&=~(INLCR|ICRNL|IGNCR);
 Opt.c_oflag&=~(ONLCR|OCRNL);
 Opt.c_lflag&=~(ICANON|ECHO|ECHOE|ISIG);
 // Opt.c_oflag &= ~OPOST;Opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

 //Opt.c_oflag  &= ~OPOST;
 Opt.c_oflag&=~OPOST;//使用原始数据


 //只有阻塞时，下面两个才有效，且针对读。
 Opt.c_cc[VMIN]=255;//最小字节数（读）   4800  50  5     这个傻意思忘记了
 Opt.c_cc[VTIME]=1;//等待时间，单位百毫秒（读）  115200 50 1


    cfsetispeed(&Opt,B115200);
    cfsetospeed(&Opt,B115200);

    tcflush(port_fd,TCIFLUSH);
    tcsetattr(port_fd,TCSANOW,&Opt);


}

int serial_port::return_port()
{
    return port_fd;
}


#endif /* Sutpc_ros_odom_hpp */
