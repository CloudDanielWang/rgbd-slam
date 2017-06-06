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
		void wifi_init_uav();	//UAV端初始化函数
		void wifi_init_pc();	//PC端初始化函数

		int send_time;		//WiFi传送数据所需时间
	
		struct sockaddr_in Server_Addr;		//服务器，即接收端地址
		struct sockaddr_in Client_Addr;		//客户端，即发送端地址

		//
		int pc_sock;
		int uav_sock;

	protected:
		//parameters
		int SERVER_PORT;		//本地端口
		int CLIENT_PORT;	//远端端口

		const char* SERVER_IP;	//本地IP
		const char* CLIENT_IP;	//远端IP

	public:
		//void send_data(char *data, unsigned int num);		//WiFi发送数据函数
		void send_data_new(Mat frame);			//WIFI 发送测试
		//int  receive_data(char *data, long unsigned int num);		//WiFi接受数据函数
		void receive_data_pc(Mat frame);			//wifi pc 接受新函数

	public:
		//data
		char red[307200];
		char green[307200];
		char blue[307200];
		char depth[76800];


};




}	//namespace acrbslam

#endif //WIFI_H