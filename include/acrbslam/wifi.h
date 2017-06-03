#ifndef WIFI_H
#define WIFI_H

#include "acrbslam/common_include.h"
#include "acrbslam/visual_odometry.h"
#include "acrbslam/config.h"
#include "acrbslam/frame.h"
#include "acrbslam/converter.h"


namespace acrbslam
{

	typedef struct
	{
		//float roll;
		//float pitch;
		//float yaw;


		float x;
		float y;
		float z;

		char *red;
		char *green;
		char *blue;
		char *depth;

		Mat rgb_mat;
		Mat depth_mat;



	}ACRB_WIFI_DATA_;



class wifi_comu:public Frame, public Converter			//继承了frame类
{
	public:
		wifi_comu();		//wifi参数读取函数
		~wifi_comu();

		void wifi_init();		//WiFi初始化函数
		void wifi_init_uav();	//新版测试函数

		int send_time;		//WiFi传送数据所需时间
	
		struct sockaddr_in Local_Addr;
		struct sockaddr_in Remote_Addr;

		//csdn测试部分
		struct hostent *server;
		//
		int pc_sock;

	protected:
		//parameters
		int LOCALPORT;		//本地端口
		int REMOTEPORT;	//远端端口

		const char* LOCALIP;	//本地IP
		const char* REMOTEIP;	//远端IP

	public:
		void send_data(char *data, unsigned int num);		//WiFi发送数据函数
		void send_data_new(Mat frame);			//WIFI 发送测试
		int  receive_data(char *data, long unsigned int num);		//WiFi接受数据函数

	public:
		//data
		char red[307200];
		char green[307200];
		char blue[307200];
		char depth[76800];


};




}	//namespace acrbslam

#endif //WIFI_H