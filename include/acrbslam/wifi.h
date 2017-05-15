#include "acrbslam/common_include.h"
#include "acrbslam/visual_odometry.h"
#include "acrbslam/config.h"
#include "acrbslam/frame.h"


//后期WIFI调通后再弄清楚获取参数的方法
#define LOCALPORT 6000          //本地IP及端口
#define LOCALIP "192.168.1.1"

#define REMOTEPORT 5000         //远程IP及端口
#define REMOTEIP "192.168.1.2"

//namespace acrbslam
//{

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

	void wifi_comu (acrbslam::Frame  *frame);
//}

