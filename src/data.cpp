#include <acrbslam/data.h>

namespace acrbslam
{

Data::Data()
{
	Mat CameraImage=Mat::zeros(480,640,CV_8UC1);	
	Mat Depth=Mat::zeros(240,320,CV_8UC1);		
	Mat ImageBlueChannel=Mat::zeros(480,640,CV_8UC1);	
	Mat ImageGreenChannel=Mat::zeros(480,640,CV_8UC1);	
	Mat ImageRedChannel=Mat::zeros(480,640,CV_8UC1);
}

Data::~Data()
{

}


void Data::inputData(Frame::Ptr frame)
{

	CameraImage=frame->color_.clone();// 初始化时第一帧记为关键帧
	Depth=frame->depth_.clone();
	T_c_w=frame->T_c_w_;    
	processingdata();

	return;

}	


void Data::processingdata()
{
	transfomation = T_c_w.matrix();
	rotation_estimate = transfomation.rotation();
	translation_estimate=transfomation.translation();

}






}//namespace acrbslam

