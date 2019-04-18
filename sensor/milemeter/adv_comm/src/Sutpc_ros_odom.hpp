//
//  Sutpc_ros_odom.hpp
//  sutpc_ros_odom
//
//  Created by 钟海兴 on 2018/12/29.
//  Copyright © 2018 钟海兴. All rights reserved.
//

#ifndef Sutpc_ros_odom_hpp
#define Sutpc_ros_odom_hpp

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
        port_fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
    if(i==3)
        port_fd = open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_NDELAY);
    if(i==4)
        port_fd = open("/dev/ttyS4", O_RDWR | O_NOCTTY | O_NDELAY);
    if(i==11)
        port_fd = open("/dev/ttyUSB008", O_RDWR | O_NOCTTY | O_NDELAY);

    if (port_fd == -1)
    {
        /*
         * Could not open the port.
         */
        perror("open_port: Unable to open /dev/ttyUSB008 -");
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

    Opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    Opt.c_oflag &= ~OPOST;

    Opt.c_cflag |= CLOCAL | CREAD;

    Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /*//8bit数据位
     Opt.c_cflag &= ~CSIZE;
     Opt.c_cflag |= CS8;
     //清除校验位
     Opt.c_cflag &= ~PARENB;
     //1bit停止位
     Opt.c_cflag &= ~CSTOPB;*/

    cfsetispeed(&Opt,B115200);
    cfsetospeed(&Opt,B115200);
    tcsetattr(port_fd,TCSANOW,&Opt);
}

int serial_port::return_port()
{
    return port_fd;
}


#endif /* Sutpc_ros_odom_hpp */
