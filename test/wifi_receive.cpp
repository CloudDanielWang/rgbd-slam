#include <fstream>
#include <boost/timer.hpp>

#include "acrbslam/config.h"
#include "acrbslam/visual_odometry.h"
#include "acrbslam/cloudmap.h"
#include "acrbslam/wifi.h"

namespace acrbslam
{


void *wifi_recv(void *arg)
{
	//struct sockaddr_in Local_Addr;
	//struct sockaddr_in Remote_Addr;
	//int pc_sock,a;
	//socklen_t addr_len=sizeof(Remote_Addr);

	wifi_comu wifi_comu_;
    	wifi_comu_.wifi_init();


	char red[307200];
	char green[307200];
	char blue[307200];
	char depth[76800];
	//char move[48];
	int a;

	Mat picture=Mat::zeros(480,640,CV_8UC3);

/*
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cur_point (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>& cloud_cur = *cloud_cur_point;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cur_trans (new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_cur.width = 19200;
	cloud_cur.height = 1;
	cloud_cur.is_dense = false;
	cloud_cur.points.resize(cloud_cur.width*cloud_cur.height);
	unsigned short int *depth_point;

	printf("OK\n");
*/
	while(1)
	{	

		a=wifi_comu_.receive_data((char*)red,sizeof(red));
		a=wifi_comu_.receive_data((char*)green,sizeof(green));
		a=wifi_comu_.receive_data((char*)blue,sizeof(blue));
		//a=wifi_comu_.receive_data(&depth,sizeof(depth));
		//a=wifi_comu_.receive_data(&transform,sizeof(transform));

		/*
		a=recvfrom(pc_sock,&red,sizeof(red),0,(struct sockaddr*)&Remote_Addr,&addr_len);//接收数据
		a=recvfrom(pc_sock,&green,sizeof(green),0,(struct sockaddr*)&Remote_Addr,&addr_len);
		a=recvfrom(pc_sock,&blue,sizeof(blue),0,(struct sockaddr*)&Remote_Addr,&addr_len);
		a=recvfrom(pc_sock,&depth,sizeof(depth),0,(struct sockaddr*)&Remote_Addr,&addr_len);
		a=recvfrom(pc_sock,&move,sizeof(move),0,(struct sockaddr*)&Remote_Addr,&addr_len);
		*/

		//depth_point = (unsigned short int *)(&depth);

		wifi_comu_.rgbchar2rgbmat(wifi_comu_, picture, red, green, blue);

		imshow("WIFI picture",picture);
		waitKey(0);

/*
		double movement[6];
		double *real_move=(double *)(move);
		movement[0]=*real_move;
		movement[1]=*(++real_move);
		movement[2]=*(++real_move);
		movement[3]=*(++real_move);
		movement[4]=*(++real_move);
		movement[5]=*(++real_move);
		printf("x:%f, y:%f, z:%f, roll:%F, pitch:%F, yaw:%f",movement[0],movement[1],movement[2],movement[3],movement[4],movement[5]);

		printf("OK3\n");

		Mat rvec=Mat::zeros(3,1,CV_64F);
		rvec.at<double>(0,0)=-(double)(movement[3]*3.1415/180.0);
		rvec.at<double>(1,0)=(double)(movement[4]*3.1415/180.0);
		rvec.at<double>(2,0)=-(double)(movement[5]*3.1415/180.0);
		Mat R;
		Rodrigues(rvec,R);
		Eigen::Matrix3d r;	

		r(0,0)=R.at<double>(0,0);
		r(0,1)=R.at<double>(0,1);
		r(0,2)=R.at<double>(0,2);
		r(1,0)=R.at<double>(1,0);
		r(1,1)=R.at<double>(1,1);
		r(1,2)=R.at<double>(1,2);
		r(2,0)=R.at<double>(2,0);
		r(2,1)=R.at<double>(2,1);
		r(2,2)=R.at<double>(2,2);

		Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
		Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();

		Eigen::AngleAxisd angle(r);
		T = angle;	

		T(0,3) =-(double)(movement[0]/1000.0);
		T(1,3) =-(double)(movement[1]/1000.0);
		T(2,3) =-(double)(movement[2]/1000.0);	

		cout<<"T:"<<T.matrix()<<endl;	

		T1=T.inverse();

		pcl::transformPointCloud(*cloud_cur_point,*cloud_cur_trans, T.matrix());
		printf("OK5\n");
		*output += *cloud_cur_trans;

		printf("OK4\n");		
	*/
	}
}

/*
void* pcl_3d(void *arg)
{
	pcl::visualization::PCLVisualizer viewer("picture");
	viewer.addPointCloud(output,"one");
	while(1)
	{
//	viewer.addPointCloud(output,"one");
//	viewer.removePointCloud("one");
	viewer.updatePointCloud(output,"one");
	viewer.spinOnce(1300);
	}
}
*/


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

