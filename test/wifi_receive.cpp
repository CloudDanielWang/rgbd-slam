#include <fstream>
#include <boost/timer.hpp>

#include "acrbslam/config.h"
//#include "acrbslam/visual_odometry.h"
//#include "acrbslam/cloudmap.h"
#include "acrbslam/wifi.h"

namespace acrbslam
{


void *wifi_recv(void *arg)
{
	wifi_comu wifi_comu_;
    	wifi_comu_.wifi_init_pc();

	Mat CameraRGBimage=Mat::zeros(480,640,CV_8UC3);

	Mat Depth;	

	while(1)
	{	
		CameraRGBimage=wifi_comu_.receive_data_pc(CameraRGBimage);
		imshow("WIFI picture",CameraRGBimage);
		waitKey(1);


	}
	close(wifi_comu_.server_sock);
}



}	//namespace acrbslam



int main(int argc, char** argv)
{
   	 if ( argc != 2 )
   	 {
       	 cout<<"usage: run_vo parameter_file"<<endl;
      	  return 1;
   	 }
    	acrbslam::Config::setParameterFile ( argv[1] );    

	pthread_t thread_wifi_recv;
	void *retval_wifi_recv;
	//pthread_t thread_pcl;
	//void *retval_pcl;

	int ret_wifi=pthread_create(&thread_wifi_recv,NULL,acrbslam::wifi_recv,NULL);
	//int ret_pcl =pthread_create(&thread_pcl,NULL,pcl_3d,NULL);

	while(1);

	pthread_join(thread_wifi_recv,&retval_wifi_recv);
	//pthread_join(thread_pcl,&retval_pcl);

	return 0;

}

