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

	cv::Mat CameraImage;	//包含在了frame 里
	Mat Depth;		//包含在了frame 里
	Mat ImageBlueChannel;
	Mat ImageGreenChannel;
	Mat ImageRedChannel;

	//Frame::Ptr keyframe = acrbslam::Frame::createFrame();


	//char *rotation;
	//char *translation;

	//char rotation_char[3][3];
	//char translation_char[3];

	char *rotation_char;
	char * translation_char;




};








}// namesapce acrbslam



#endif	//DATA_H