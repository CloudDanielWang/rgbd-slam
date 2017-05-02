#ifndef H_wifi
#define H_wifi

#include <stdio.h>      /*标准输入输出定义*/
#include <iostream>
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h> 
#include <sys/stat.h>  
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*POSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <pthread.h>
#include <string.h> //和strings.h有什么区别

#include <linux/i2c-dev.h>  
#include <sys/ioctl.h> 
#include <signal.h>
#include <sys/time.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <stddef.h>  
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <getopt.h>
#include <fcntl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "stddef.h"
#include <malloc.h>

#include "slamBase.h"


#define LOCALPORT 6000          //本地IP及端口
#define LOCALIP "192.168.0.132"

#define REMOTEPORT 5000         //远程IP及端口
#define REMOTEIP "192.168.0.166"

//wifi数据结构
typedef struct
{
	cv::Mat rvec;
	cv::Mat tvec;
	float time;
	cv::Mat cloudMap;




}ACRB_WIFI_DATA;



struct SLAM_DATA
{
	struct FRAME FRAME;
	struct RESULT_OF_PNP RESULT_OF_PNP;
};

#endif
