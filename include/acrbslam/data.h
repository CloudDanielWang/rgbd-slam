#ifndef DATA_H
#define DATA_H

#include <acrbslam/common_include.h>
#include <acrbslam/frame.h>

namespace acrbslam
{


class Data
{
public:
	char red_temp[307200];
	char green_temp[307200];
	char blue_temp[307200];

	char *red;
	char *green;
	char *blue;
	char *depth;

	Mat CameraImage;	
	Mat Depth;		
	Mat ImageBlueChannel;
	Mat ImageGreenChannel;
	Mat ImageRedChannel;

	SE3 T_c_w;
	Eigen::Isometry3d transfomation;
	Eigen::Matrix3d rotation_estimate;
	Eigen::Vector3d translation_estimate;

	char *rotation_char;
	char * translation_char;

public:
	Data();
	~Data();

	void inputData(Frame::Ptr frame);		//将frame中的参数保存在data类中

protected:
	void processingdata();




};








}// namesapce acrbslam



#endif	//DATA_H