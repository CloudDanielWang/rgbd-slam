#include <fstream>
#include <boost/timer.hpp>

#include "acrbslam/config.h"
//#include "acrbslam/visual_odometry.h"
//#include "acrbslam/cloudmap.h"
#include "acrbslam/converter.h"
#include "acrbslam/wifi.h"
#include "acrbslam/data.h"

#include "acrbslam/viz.h"

namespace acrbslam
{
void *wifi_recv(void *arg);
void *viz_thread(void *arg);
//viz::Viz3d viz_initialize();		
//void viz_update(viz::Viz3d vis);

}//namespace acrbslam


acrbslam::Data data;	//数组初始化；
sem_t sem;


int main(int argc, char** argv)
{
   	 if ( argc != 2 )
   	 {
       	 cout<<"usage: run_vo parameter_file"<<endl;
      	  return 1;
   	 }
    	acrbslam::Config::setParameterFile ( argv[1] );    

    	    //初始化信号量，其初值为0  
    	if((sem_init(&sem, 0, 0)) == -1)  
    	{  
      	perror("semaphore intitialization failed\n");  
       	exit(EXIT_FAILURE);  
    	} 


	pthread_t thread_wifi_recv;	void *retval_wifi_recv;
	pthread_t thread_viz_show;	void *retval_viz_show;
	//pthread_t thread_pcl;
	//void *retval_pcl;

	int ret_wifi=pthread_create(&thread_wifi_recv,NULL,acrbslam::wifi_recv,NULL);
	int ret_viz=pthread_create(&thread_viz_show, NULL,acrbslam::viz_thread, NULL);
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
    	//viz::Viz3d viz= viz_initialize();
    	int receive_count=0;
    	
	while(1)
	{	
	//CameraRGBimage=wifi_comu_.receive_data_pc(CameraRGBimage);	//该函数使用正常，不错乱
	if(wifi_comu_.receive_data_server_readv(&data.CameraImage, &data.Depth, &data.T_c_w_mat))
	{	
		sem_post(&sem);//信号量加1

		receive_count++;
		cout<<"receive_count:"<<receive_count<<endl;

		imshow("WIFI RGBData",data.CameraImage);
		//imshow("WIFI DepthData", Depth);
		waitKey(1);
		//data.T_c_w=data.toSE3(data.T_c_w_mat);
		//cout<<"TCW"<<data.T_c_w.matrix()<<endl;
		//viz_update(viz);
		continue;
	}
	

	}
	close(wifi_comu_.server_sock);
}


/*
viz::Viz3d viz_initialize()
{

	sleep(1);
	cv::viz::Viz3d vis ( "Visual Odometry" );
    	cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    	cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    	cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    	vis.setViewerPose ( cam_pose );

    	world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    	camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
   	 vis.showWidget ( "World", world_coor );
    	vis.showWidget ( "Camera", camera_coor );

    	return  vis;

}

void viz_update(viz::Viz3d vis)
{
    	cv::Affine3d M;
    	data.T_c_w=data.toSE3(data.T_c_w_mat);
    	SE3 Twc = data.T_c_w.inverse();
    	M=data.toAffine3d(Twc);
    	vis.setWidgetPose ( "Camera", M );
    	vis.spinOnce ( 1, false );
    	return;
}
*/
void *viz_thread(void *arg)
{
	viz::Viz3d viz= viz_initialize();
	while(1)
	{
		sem_wait(&sem); 
		viz_update(viz,data);

	}


}

   



}	//namespace acrbslam