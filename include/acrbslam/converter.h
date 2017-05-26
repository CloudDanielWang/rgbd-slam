#ifndef CONVERTER_H
#define CONVERTER_H

#include "acrbslam/common_include.h"

namespace acrbslam
{


class Converter
{
public:
	//Converter();
	//~Converter();

	void rgbmat2rgbchar(Converter converter, Mat rgbmat, char **red,char **green,char **blue);	//将RGB图像转化为便于WiFi传输的char*
	
	void mat2char(Converter converter, Mat mat_, char** char_);


	char red_mat[307200];
	char green_mat[307200];
	char blue_mat[307200];
	unsigned short int depth_mat[307200];

	//char *pic_red,*pic_green,*pic_blue,*pic_depth;
};






}//namespace arbslam

#endif //CONVERTER_H