#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include <sstream>
#include <cstdlib>
#include "stdio.h"
#include <unistd.h>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include<string.h>
#include <errno.h>

using namespace std;

#define MAXSIZE 1024 
#define QUEUE   20 
#define lamp_buf_len 76

    int sock,accept_fd;
    string clientmsg;
    std_msgs::String msg;
    char light;
    char ltime;
    char buffer[MAXSIZE];
    int bufend;
    string server_ip="192.168.110.1";
    int server_port=9999;
    int connectdelay=0;

/*
class tcp_server
{
private:
    int socket_fd,accept_fd;
    sockaddr_in myserver;
    sockaddr_in remote_addr;
    string clientmsg;
    std_msgs::String msg;
    char light;
    char time;
    char buffer[MAXSIZE];
    int bufend;
public:
    tcp_server();
    ~tcp_server();
    int recv_msg();
    string returnmsg();
    void closeacpt();
    char bcc_check(char *buf,int start,int len);
    char getlight();
    char gettime();
};

tcp_server::tcp_server() 
{  
    if(( socket_fd = socket(PF_INET,SOCK_STREAM,IPPROTO_TCP)) < 0 )
    {  
        throw "socket() failed";  
    }    
    memset(&myserver,0,sizeof(myserver));  
    myserver.sin_family = AF_INET;  
    myserver.sin_addr.s_addr = htonl(INADDR_ANY);  
    myserver.sin_port = htons(MYPORT);  
  
    if( bind(socket_fd,(sockaddr*) &myserver,sizeof(myserver)) < 0 ) 
    {  
        throw "bind() failed";  
    }  
  
    if( listen(socket_fd,10) < 0 ) 
    {  
        throw "listen() failed";  
    } 
    socklen_t sin_size = sizeof(struct sockaddr_in);  
    if(( accept_fd = accept(socket_fd,(struct sockaddr*) &remote_addr,&sin_size)) == -1 )  
    {  
        throw "Accept error!";  
    }
    else
        ROS_INFO("Received a connection from %s\n",(char*) inet_ntoa(remote_addr.sin_addr)); 
    bufend=0; 
    light=0;
    time=0;
    memset(&buffer,0,sizeof(buffer));    
}  
tcp_server::~tcp_server()
{
    close(accept_fd);  
    close(socket_fd);
}
//get signal from lamp 
int tcp_server::recv_msg()
{   
    {
        int i=0;
	char tempbuf[MAXSIZE];
        memset(&tempbuf,0,sizeof(tempbuf));
        int length=recv(accept_fd,tempbuf,MAXSIZE,0);
        ROS_INFO("Received message: %x %x... length : %d \n", tempbuf[0],tempbuf[1] , length); 	

	for(i=0; i<length; i++ ){
	    buffer[bufend]=tempbuf[i];
	    bufend++;
	}
	if(bufend>=lamp_buf_len)
	{
	    for(i=0;i<=bufend-lamp_buf_len;i++)
	    { 
		if(buffer[i]==0xAA&&buffer[i+1]==0xBB)
		{
		    if(bcc_check(buffer,i,lamp_buf_len-1)==buffer[i+lamp_buf_len-1])
		    {
			light=buffer[i+6];
			time =buffer[i+7];
			bufend=0;
		    }
		}
	    }
	} 
    }   
    return 0;
}  

char tcp_server::bcc_check(char *buf,int start,int len)
{
    
    char checksum=0;
    for(int i=start; i<start+len; i++) checksum^=buf[i];
    return checksum;
}

string tcp_server::returnmsg()
{
    return clientmsg;
}
char tcp_server::getlight()
{
    return light;   
}
char tcp_server::gettime()
{
    return time;
}

void tcp_server::closeacpt()
{
    close(accept_fd);
}
*/


char bcc_check(char *buf,int start,int len)
{
    
    char checksum=0;
    for(int i=start; i<start+len; i++) checksum^=buf[i];
    return checksum;
}


void client()
{
    struct sockaddr_in serv_addr;
    struct timeval timeout; 
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1)
        throw "socket error!";
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(server_ip.c_str());
    serv_addr.sin_port = htons(server_port);

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1)
        ROS_INFO("not connected");
    timeout.tv_sec=0;   
    timeout.tv_usec=5000;
    int result = setsockopt(sock,SOL_SOCKET,SO_RCVTIMEO,(char *)&timeout.tv_sec,sizeof(struct timeval));
    if (result < 0)
    {
      perror("setsockopt"); 
    }
}
void reconnect()
{

    struct sockaddr_in serv_addr;
    struct timeval timeout;
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1)
        throw "socket error!";
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(server_ip.c_str());
    serv_addr.sin_port = htons(server_port);
    ROS_INFO("port :%d",server_port);
    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1)
        ROS_INFO("not connected");
    timeout.tv_sec=0;
    timeout.tv_usec=5000;
    int result = setsockopt(sock,SOL_SOCKET,SO_RCVTIMEO,(char *)&timeout.tv_sec,sizeof(struct timeval));
    if (result < 0)
    {
      perror("setsockopt");
    }
	
}

void rmsg()
{
    {
        int i=0;
        char tempbuf[MAXSIZE];
        memset(&tempbuf,0,sizeof(tempbuf));


        int sockErr = errno;
        int lengthflag=recv(sock,tempbuf,MAXSIZE,MSG_PEEK);
	if(lengthflag>0)
	{
	    connectdelay=0;
	    ROS_INFO("Received message length: %d\n",lengthflag); 
	} 
	else if((lengthflag==-1)&&(sockErr == EWOULDBLOCK)&&(connectdelay<=200))
    	{
	    connectdelay++;
	    //ROS_INFO("No message socket alive");
            //No data received
    	}
    	else
    	{
	    ROS_INFO("Socket closed");
       	    close(sock);
	    //ROS_INFO("reconnect");
            reconnect();
    	}

        int length=recv(sock,tempbuf,MAXSIZE,0);
        //ROS_INFO("Received message: %x %x... length : %d \n", tempbuf[11],tempbuf[12] , length);

        for(i=0; i<length; i++ ){
            buffer[bufend]=tempbuf[i];
            bufend++;
        }
        if(bufend>=lamp_buf_len)
        {
            for(i=0;i<=bufend-lamp_buf_len;i++)
            {
                if(buffer[i]==0xAA&&buffer[i+1]==0xBB)
                {
                    if(bcc_check(buffer,i,lamp_buf_len-1)==buffer[i+lamp_buf_len-1])
                    {
                        light=buffer[i+11];
                        ltime =buffer[i+12];
                        bufend=0;
			ROS_INFO("light color: %x ,time: %x \n",light,ltime);
                    }
                }
            }
        }
    }
}


int main(int argc,char** argv)
{
    ros::init(argc, argv, "signal_lamp");

    ros::NodeHandle n;
    ros::Publisher lamp_signal;
    lamp_signal = n.advertise<std_msgs::UInt8MultiArray>("lamp_signal", 1000);
    ros::NodeHandle nh_private("~");
    ros::Rate loop_rate(10);
    std_msgs::UInt8MultiArray m;

    nh_private.param<int>("server_port", server_port, 9999);
    nh_private.param<string>("server_ip", server_ip, "192.168.110.1");

    client();

    m.layout.dim.push_back(std_msgs::MultiArrayDimension());
    m.layout.dim[0].size = 2;
    m.layout.dim[0].stride = 1;
    m.layout.dim[0].label = "lamp_m";
    m.data.resize(2);


    light=0x04;
    ltime=0x01;

    while(ros::ok())
    {
        //ts.recv_msg();
        rmsg();


        m.data[0] =light;
        m.data[1] =ltime;

        //ROS_INFO("light %x,time %x", m.data[0],m.data[1]);
        lamp_signal.publish(m);
        ros::spinOnce();
        /*test*/
        loop_rate.sleep();
    }

    close(sock);
    //ts.closeacpt();
    return 0;
}


