#include <fstream>
#include <boost/timer.hpp>

#include "acrbslam/config.h"
//#include "acrbslam/visual_odometry.h"
//#include "acrbslam/cloudmap.h"
#include "acrbslam/converter.h"
#include "acrbslam/wifi.h"
#include "acrbslam/data.h"

namespace acrbslam
{
void *wifi_recv(void *arg);
void *viz_thread(void *arg);

}	//namespace acrbslam

	acrbslam::Data data;	//数组初始化；

int main(int argc, char** argv)
{
   	 if ( argc != 2 )
   	 {
       	 cout<<"usage: run_vo parameter_file"<<endl;
      	  return 1;
   	 }
    	acrbslam::Config::setParameterFile ( argv[1] );    




	pthread_t thread_wifi_recv;	void *retval_wifi_recv;
	pthread_t thread_viz_show;	void *retval_viz_show;
	//pthread_t thread_pcl;
	//void *retval_pcl;

	int ret_wifi=pthread_create(&thread_wifi_recv,NULL,acrbslam::wifi_recv,NULL);
	//int ret_viz=pthread_create(&thread_viz_show, NULL,acrbslam::viz_thread, NULL);
	//int ret_pcl =pthread_create(&thread_pcl,NULL,pcl_3d,NULL);

	while(1);

	pthread_join(thread_wifi_recv,&retval_wifi_recv);
	pthread_join(thread_viz_show,&retval_viz_show);
	//pthread_join(thread_pcl,&retval_pcl);

	return 0;

}






namespace acrbslam
{


void *wifi_recv(void *arg)
{
	wifi_comu wifi_comu_;
    	wifi_comu_.wifi_init_pc();	 
    	
	while(1)
	{	
	//Mat CameraRGBimage=Mat::zeros(480,640,CV_8UC3);
	//Mat Depth=Mat::zeros(480,640,CV_16UC1);
	//Mat transformation=Mat::zeros(4,4,CV_32F);
		//CameraRGBimage=wifi_comu_.receive_data_pc(CameraRGBimage);	//该函数使用正常，不错乱
	if(wifi_comu_.receive_data_server_readv(&data.CameraImage, &data.Depth, &data.T_c_w_mat))
	{
		imshow("WIFI RGBData",data.CameraImage);
		//imshow("WIFI DepthData", Depth);
		waitKey(1);
		data.T_c_w=wifi_comu_.toSE3(data.T_c_w_mat);
		cout<<"TCW"<<data.T_c_w.matrix()<<endl;
		continue;
	}
	

	}
	close(wifi_comu_.server_sock);
}



void *viz_thread(void *arg)
{
while(1)
{	
	sleep(1);
	cv::viz::Viz3d vis ( "Visual Odometry" );
    	cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    	cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    	cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    	vis.setViewerPose ( cam_pose );

    	world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    	camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
   	 vis.showWidget ( "World", world_coor );
    	vis.showWidget ( "Camera", camera_coor );

    	cv::Affine3d M;
    	M=data.toAffine3d(data.T_c_w);
    	vis.setWidgetPose ( "Camera", M );
    	vis.spinOnce ( 1, false );
}

   
}//viz_thread


}	//namespace acrbslam