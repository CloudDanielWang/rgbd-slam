#include <fstream>
#include <boost/timer.hpp>

#include "acrbslam/config.h"
#include "acrbslam/visual_odometry.h"
#include "acrbslam/cloudmap.h"
#include "acrbslam/wifi.h"
#include "acrbslam/data.h"

namespace acrbslam
{
    acrbslam::Data data;    //数据存储类 

    void* vo_thread(void *arg);
    void* wifi_thread(void *arg);

    //void *pointcloud_thread(void *arg);
}   //namespace acrbslam 
   
    pthread_mutex_t mutex_data; //互斥锁
sem_t sem_TCP;



int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }
    acrbslam::Config::setParameterFile ( argv[1] );     
    
    int res=pthread_mutex_init(&mutex_data, NULL);
    if (res!=0)
    {
        perror("Mutex initialization failed");
        exit(EXIT_FAILURE);
    }
        //初始化TCP信号量，其初值为0  
        if((sem_init(&sem_TCP, 0, 0)) == -1)  
        {  
        perror("semaphore of TCP intitialization failed\n");  
        exit(EXIT_FAILURE);  
        } 

    pthread_t   thread_wifi;        void *retval_wifi;
    pthread_t   thread_vo;          void *retval_vo;

/*
//test cloudmap thread
    pthread_t thread_pointcloud;    void *retval_pointcloud;
    int ret_pointcloud=pthread_create(&thread_pointcloud,NULL,acrbslam::pointcloud_thread,NULL);
  */  


    int ret_vo=pthread_create(&thread_vo,NULL,acrbslam::vo_thread,NULL);
    if(ret_vo !=0)
    {
        perror("VO Thread Create  Failed");
        exit(EXIT_FAILURE);
    }

    int ret_wifi=pthread_create(&thread_wifi,NULL, acrbslam::wifi_thread, NULL);
    if(ret_wifi !=0)
    {
        perror("WIFI Thread Create  Failed");
        exit(EXIT_FAILURE);
    }

    while(1);
    
    pthread_join(thread_vo,&retval_vo);
    pthread_join(thread_wifi,&retval_wifi);

    //pthread_join(thread_pointcloud,&retval_pointcloud);
      

    return 0;
}





namespace acrbslam
{

void* vo_thread(void *arg)
{
    acrbslam::VisualOdometry::Ptr vo ( new acrbslam::VisualOdometry );
    acrbslam::Camera::Ptr camera ( new acrbslam::Camera );


    //openni 输入
    //VideoCapture capture(CV_CAP_OPENNI);    //设置视频的来源为OPENNI设备，即Kinect


//数据集文件的读取部分
//
    string dataset_dir = acrbslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        //return;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
                 break;

    }
//数据集读取结束


    ///
    //wifi_comu wifi_comu_;
   // wifi_comu_.wifi_init_uav();

    ///


       for ( int i=0; i<vo->scan_frame_num_ ; i++ )       //循环次数取决与参数文件中的数值
      // for ( int i=0; i<rgb_files.size(); i++ )           //循环次数取决与文件帧的多少
    {
               
        cout<<"****** loop "<<i<<" ******"<<endl;
        //需要切换为OPENNI设备时，将下面部分注释取消
        // capture.grab();  
        //Mat color;
       // Mat depth ;
       // capture.retrieve( color, CV_CAP_OPENNI_BGR_IMAGE );             //数据的读取可以做多线程加队列进行优化，加快运行速度（目前未优化）
        //capture.retrieve( depth, CV_CAP_OPENNI_DEPTH_MAP ); 
        //OPENNI END

        //数据来源为DATA文件夹下的TUM数据集
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            break;
        //TUM end

        ///

        acrbslam::Frame::Ptr pFrame = acrbslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;

        //数据集时所用时间戳
        pFrame->time_stamp_ = rgb_times[i];
        

        boost::timer timer;
       // vo->addFrame ( pFrame );
        pthread_mutex_lock( &mutex_data );      //对data互斥锁住

        data=vo->addFrame(pFrame, data);
        data.frameID=i;
        int data_empty_flag=data.CameraImage.empty();
        data.End_Flag='0';
        if(i==vo->scan_frame_num_-1 ) {data.End_Flag='1';}
        //cout<<"VO END flag"<<data.End_Flag<<endl;

        pthread_mutex_unlock( &mutex_data);     //对data解锁
        sem_post(&sem_TCP);//信号量加1

        if(data_empty_flag==0)
        {
            cout<<"This Frame is Not The KeyFrame!!!"<<endl;
        }


        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == acrbslam::VisualOdometry::LOST )
            break;
    }
    //return;    //如何关闭线程？？
        cout<<"VO Thread Exit"<<endl;

}


void* wifi_thread(void *arg)
{   
    wifi_comu wifi_comu_;
    wifi_comu_.wifi_init_uav();
    usleep(100);

    while(1)
    {    
        sem_wait(&sem_TCP); 
        pthread_mutex_lock(&mutex_data);        //对data互斥线程锁，以避免在对data判断期间VO线程的干扰
        int flag=data.CameraImage.empty();
        if(flag==0)
         {  
        //Eigen::Matrix4d translation=data.toMatrix4d(data.T_c_w_mat);
        //cout<<"translation"<<translation<<endl;
       // cout<<"wifi send data begin"<<endl;
        //wifi_comu_.send_data_client_writev(data.CameraImage, data.Depth, data.T_c_w_mat);
        //wifi_comu_.send_data_client_writev(data.CameraImage, data.Depth, data.T_c_w_mat, data.End_Flag);
        //cout<<"wifi send data finish"<<endl;
        //wifi_comu_.send_data_new(data.CameraImage);
        //wifi_comu_.send_data_new(data.Depth);
            /**********************************************************************/
            data.RGBImgSize = data.CameraImage.total()*data.CameraImage.elemSize();     
            data.DepthImgSize =data.Depth.total()*data.Depth.elemSize();        
            data.TransMatrixSize =data.T_c_w_mat.total()*data.T_c_w_mat.elemSize();     
            data.EndFlagSize = sizeof(data.End_Flag);  
                      
            data.TCPSendDataSize=data.RGBImgSize+data.DepthImgSize+data.TransMatrixSize+data.EndFlagSize;                                
/*
            //uchar TCPRGBData[data.RGBImgSize];
            //uchar TCPDepthData[data.DepthImgSize];
            uchar* TCPRGBData;
            uchar* TCPDepthData;
            uchar* TCPTransData;
            uchar* TCPEndFlagData;

            //uchar* TCPSendData;
            uchar TCPSendData[data.TCPSendDataSize];
            

            TCPRGBData=data.CameraImage.data;
            TCPDepthData=data.Depth.data;
            TCPTransData=data.T_c_w_mat.data;
            //TCPEndFlagData=(uchar *)data.End_Flag;
            //cout<<"TCPRGBData AND TCPDepthData 赋值完成"<<endl;

            memcpy(TCPSendData,TCPRGBData,data.RGBImgSize);
            memcpy(TCPSendData+data.RGBImgSize,TCPDepthData,data.DepthImgSize);
            memcpy(TCPSendData+data.RGBImgSize+data.DepthImgSize,TCPTransData,data.TransMatrixSize);
            //memcpy(TCPSendData+data.RGBImgSize+data.DepthImgSize+data.TransMatrixSize,TCPEndFlagData,data.EndFlagSize);
            TCPSendData[data.RGBImgSize+data.DepthImgSize+data.TransMatrixSize]=data.End_Flag;
   
            //cout<<"TCPSendData 赋值完成"<<endl;

            wifi_comu_.SendTCPDataClient((uchar *)TCPSendData, data.TCPSendDataSize);
            */
            wifi_comu_.SendTCPDataClient(data);
            /*******************************************************************************/
            cout<<"frameID:\t"<<data.frameID<<endl;
            cout<<"End_Flag:\t"<<data.End_Flag<<endl;
            cout<<"TCW\n"<<data.T_c_w.matrix()<<endl;
            if(data.End_Flag=='1') 
                break;
 
            

       // cv::imshow("wifi_send thread frame",data.CameraImage);
        //cv::waitKey(1);
       
         }//endif 
        pthread_mutex_unlock(&mutex_data);      //互斥锁解锁

        usleep(100);
        //sleep(1);
        
    }
    //return;
        //pthread_mutex_unlock(&mutex_data);      //线程结束后，将互斥锁解锁，便于WiFi的最后一次发送
    exit(1);
}


/*
void *pointcloud_thread(void *arg)
{
     pointCloud::Ptr pointCloud_all( new pointCloud ); //存放所有点云
        pcl::visualization::CloudViewer viewer("cloudmap viewer");    //在线显示，不推荐开启

     while(1)
     {
         pthread_mutex_lock(&mutex_data);        //对data互斥线程锁，以避免在对data判断期间VO线程的干扰
        //draw cloudmap
                pointCloud_all=createPointCloud(data, pointCloud_all);  
                 viewer.showCloud( pointCloud_all );      //在线显示，不推荐开启
                pthread_mutex_unlock(&mutex_data);  
                usleep(10);
                //判断完成接收，退出循环，保存点云数据的语句：
                if(data.End_Flag=='1') break;

     }
    cout<<"点云大小为："<<pointCloud_all->size()<<"个点."<<endl;
    cout<<"Saving..."<<endl;
             pcl::io::savePCDFileBinary( "data/result.pcd", *pointCloud_all );
             cout<<"Point Cloud Saving Finished"<<endl;
             while( !viewer.wasStopped() )
            {}
             sleep(1);
             exit(1);

}
*/

}   //namespace acrbslam

