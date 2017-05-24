#ifndef WIFI_H
#define WIFI_H

#include "acrbslam/common_include.h"
#include "acrbslam/visual_odometry.h"
#include "acrbslam/config.h"
#include "acrbslam/frame.h"
#include "acrbslam/converter.h"


//后期WIFI调通后再弄清楚获取参数的方法
#define LOCALPORT 6000          //本地IP及端口
#define LOCALIP "192.168.1.105"

#define REMOTEPORT 5000         //远程IP及端口
#define REMOTEIP "192.168.1.103"

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

		float r;
		float g;
		float b;

		float depth;


	}ACRB_WIFI_DATA;



class wifi_comu:public Frame, public Converter			//继承了frame类
{
	public:
		wifi_comu(){};
		void wifi_init();		//WiFi初始化函数


		void send_data(char *data,unsigned int num);		//WiFi发送数据函数
		void receive_data(char *data,unsigned int num){};		//WiFi接受数据函数

		int send_time;						//WiFi传送数据所需时间
	
		struct sockaddr_in Local_Addr;
		struct sockaddr_in Remote_Addr;
		int pc_sock;
};




}	//namespace acrbslam

#endif //WIFI_H