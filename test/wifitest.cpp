#include "acrbslam/common_include.h"
#include "acrbslam/wifi.h"
#include "acrbslam/frame.h"
#include "acrbslam/converter.h"
#include "acrbslam/camera.h"


namespace acrbslam
{


void* wifi_thread(void *arg)
{	
	wifi_comu wifi_comu_;
	wifi_comu_.wifi_init();
	cv::Mat rgb=imread("./data/beauty.jpg");
	cv::imshow ( "image", rgb );
	cv::waitKey ( 0 );
	while(1)
	{	
		wifi_comu_.rgbmat2rgbchar(wifi_comu_,rgb, &wifi_comu_.pic_red, &wifi_comu_.pic_green, &wifi_comu_.pic_blue);
		//wifi_comu_.mat2char(depth, &wifi_comu_.pic_depth);	//test no depth

		wifi_comu_.send_data(wifi_comu_.pic_red,307200);
	//	printf("OK\n");
		wifi_comu_.send_data(wifi_comu_.pic_green,307200);
		wifi_comu_.send_data(wifi_comu_.pic_blue,307200);
		wifi_comu_.send_data(wifi_comu_.pic_depth,307200);

		printf("OK\n");
		
	}
}


}		//namespace acrbslam


int main(int argc, char **argv)
{
	acrbslam::Config::setParameterFile ( argv[1] );
	
	pthread_t   thread_wifi;
	void *retval_wifi;
	int ret_wifi=pthread_create(&thread_wifi,NULL, acrbslam::wifi_thread, NULL);
	while(1);
	pthread_join(thread_wifi,&retval_wifi);
	return 0;
}
