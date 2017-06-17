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
}   //namespace acrbslam 
   
    pthread_mutex_t mutex_data; //互斥锁




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


    pthread_t   thread_wifi;        void *retval_wifi;
    pthread_t   thread_vo;          void *retval_vo;


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

        data=vo->addFrame(pFrame);
        data.frameID=i;
        int data_empty_flag=data.CameraImage.empty();

        pthread_mutex_unlock( &mutex_data);     //对data解锁


        if(data_empty_flag==0)
        {
            cout<<"This Frame is Not The KeyFrame!!!"<<endl;
        }
        //wifi_comu_.send_data_client_writev(data.CameraImage, data.Depth);
        // cout<<"wifi send data finish"<<endl;
        //cout<<data.Depth.size()<<endl;
        //cv::imshow ( "VOFRAME",  data.CameraImage);
        //cv::imshow ( "depth",  data.Depth);
       // cv::waitKey ( 0);

        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == acrbslam::VisualOdometry::LOST )
            break;
    }

}


void* wifi_thread(void *arg)
{   
    wifi_comu wifi_comu_;
    wifi_comu_.wifi_init_uav();
    usleep(100);

    while(1)
    {    
        
        pthread_mutex_lock(&mutex_data);        //对data互斥线程锁，以避免在对data判断期间VO线程的干扰
        int flag=data.CameraImage.empty();
        if(flag==0)
         {  

        //cv::imshow("frame",data.CameraImage);
        //cv::waitKey(0);
        //Eigen::Matrix4d translation=data.toMatrix4d(data.T_c_w_mat);
        //cout<<"translation"<<translation<<endl;
       // cout<<"wifi send data begin"<<endl;
        wifi_comu_.send_data_client_writev(data.CameraImage, data.Depth, data.T_c_w_mat);
        //cout<<"wifi send data finish"<<endl;
        cout<<"frameID:"<<data.frameID<<endl;


       // cv::imshow("wifi_send thread frame",data.CameraImage);
        //cv::waitKey(1);
       
         }//endif 
        pthread_mutex_unlock(&mutex_data);      //互斥锁解锁

        usleep(10);
        
    }
}



}   //namespace acrbslam

