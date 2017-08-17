#ifndef GPS_H
#define GPS_H

#include "acrbslam/common_include.h"
#include "acrbslam/data.h"
#include "acrbslam/config.h"
#include "acrbslam/timer.h"

namespace acrbslam
{

class GPS
{

	public:

	GPS();

	void gps_comu(Data data);	//数据传输函数

	int gps_com_init(int fd);    //配置串口,串口初始化

	const char* GPS_com_addr;	//gps输出串口文件位置
	

	protected: 
	double calcumin(double degree);

	double calcudeg(double degree);


};




}//namespace acrbslam

#endif//GPS_H