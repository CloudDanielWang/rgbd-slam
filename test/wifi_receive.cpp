#include <fstream>
#include <boost/timer.hpp>

#include "acrbslam/config.h"
#include "acrbslam/cloudmap.h"
#include "acrbslam/converter.h"
#include "acrbslam/wifi.h"
#include "acrbslam/data.h"

#include "acrbslam/viz.h"

namespace acrbslam
{
void *wifi_recv(void *arg);
void *viz_thread(void *arg);
void *pointcloud_thread(void *arg);

}//namespace acrbslam


acrbslam::Data data;	//数组初始化；
sem_t sem_viz;
sem_t sem_cloud;



int main(int argc, char** argv)
{
   	 if ( argc != 2 )
   	 {
       	 cout<<"usage: run_vo parameter_file"<<endl;
      	  return 1;
   	 }
    	acrbslam::Config::setParameterFile ( argv[1] );    

    	//初始化VIZ信号量，其初值为0  
    	if((sem_init(&sem_viz, 0, 0)) == -1)  
    	{  
      	perror("semaphore of VIZ  intitialization failed\n");  
       	exit(EXIT_FAILURE);  
    	} 
	//初始化VIZ信号量，其初值为0  
    	if((sem_init(&sem_cloud, 0, 0)) == -1)  
    	{  
      	perror("semaphore of Cloud intitialization failed\n");  
       	exit(EXIT_FAILURE);  
    	} 


	pthread_t thread_wifi_recv;	void *retval_wifi_recv;
	pthread_t thread_viz_show;	void *retval_viz_show;
	pthread_t thread_pointcloud;	void *retval_pointcloud;


	int ret_wifi=pthread_create(&thread_wifi_recv,NULL,acrbslam::wifi_recv,NULL);
	//int ret_viz=pthread_create(&thread_viz_show, NULL,acrbslam::viz_thread, NULL);
	int ret_pointcloud=pthread_create(&thread_pointcloud,NULL,acrbslam::pointcloud_thread,NULL);

	while(1);

	pthread_join(thread_wifi_recv,&retval_wifi_recv);
	pthread_join(thread_viz_show,&retval_viz_show);
	pthread_join(thread_pointcloud,&retval_pointcloud);

	return 0;

}






namespace acrbslam
{

void *viz_thread(void *arg)
{
	viz::Viz3d viz= viz_initialize();
	while(1)
	{
		sem_wait(&sem_viz); 
		viz_update(viz,data);

		//if(data.End_Flag==1) break;
	}
}



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
	//if(wifi_comu_.receive_data_server_readv(&data.CameraImage, &data.Depth, &data.T_c_w_mat, &data.End_Flag))
	{	
		sem_post(&sem_viz);//信号量加1
		sem_post(&sem_cloud);	//点云线程信号量加1

		receive_count++;
		cout<<"receive_count:"<<receive_count<<endl;

		 cout<<"data.End_Flag:"<<data.End_Flag<<endl;

		imshow("WIFI RGBData",data.CameraImage);
		//imshow("WIFI DepthData", Depth);
		waitKey(1);
		//data.T_c_w=data.toSE3(data.T_c_w_mat);
		//cout<<"TCW"<<data.T_c_w.matrix()<<endl;
		
		//if(data.End_Flag==1) break;
		//else
		continue;
	}
	

	}
	close(wifi_comu_.server_sock);
}

void *pointcloud_thread(void *arg)
{
	 pointCloud::Ptr pointCloud_all( new pointCloud ); //存放所有点云
   	  	//pcl::visualization::CloudViewer viewer("cloudmap viewer");	//在线显示，不推荐开启

	 while(1)
	 {
	 	sem_wait(&sem_cloud);
		//draw cloudmap
        		pointCloud_all=createPointCloud(data, pointCloud_all);	
        		// viewer.showCloud( pointCloud_all );	 	//在线显示，不推荐开启

        		//判断完成接收，退出循环，保存点云数据的语句：
        		//if(data.End_Flag==1) break;

	 }
	cout<<"点云大小为："<<pointCloud_all->size()<<"个点."<<endl;
	cout<<"Saving..."<<endl;
        	 pcl::io::savePCDFileBinary( "data/result.pcd", *pointCloud_all );
        	 cout<<"Point Cloud Saving Finished"<<endl;
         	// while( !viewer.wasStopped() )
         	//{}
        	 sleep(1);
        	 exit(1);

}


   



}	//namespace acrbslam