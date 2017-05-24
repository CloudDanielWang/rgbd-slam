

#include "acrbslam/converter.h"


namespace acrbslam
{

//converter::converter();
//Converter::Converter()
//:()

void Converter::rgbmat2rgbchar(Converter converter, Mat rgbmat, char **red,char **green,char **blue)
{	

	int x,y,temp;
	for( y = 0; y < rgbmat.rows; y++ ) 
	{ 
		temp=y*rgbmat.cols;

		for( x = 0; x < rgbmat.cols; x++ ) 
		{
			converter.red_mat[temp+x]=rgbmat.at<Vec3b>(y,x)[0];
			converter.green_mat[temp+x]=rgbmat.at<Vec3b>(y,x)[1];
			converter.blue_mat[temp+x]=rgbmat.at<Vec3b>(y,x)[2];
		}
	}

	*red=converter.red_mat;
	*green=converter.green_mat;
	*blue=converter.blue_mat;
}

void Converter::mat2char(Converter converter, Mat mat_, char** char_)
{	
	int x,y,temp;
	for( y = 0; y < mat_.rows; y++ ) 
	{ 
		temp=y*mat_.cols;

		for( x = 0; x < mat_.cols; x++ ) 
		{
			converter.depth_mat[temp+x]=mat_.at<ushort>(y,x);
		}
	}

	*char_=(char *)converter.depth_mat;	
}



}//namespace acrbslam


