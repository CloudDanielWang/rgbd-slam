#include <fstream>
#include <boost/timer.hpp>

#include "acrbslam/config.h"
#include "acrbslam/visual_odometry.h"
#include "acrbslam/cloudmap.h"
#include "acrbslam/wifi.h"
#include "acrbslam/data.h"

namespace acrbslam
{
ACRB_WIFI_DATA_  ACRB_WIFI_DATA;

void* vo_thread(void *arg)
{
    //int x  =Config::get<int>("scan_frame_num");
    //cout<<x<<endl;

    acrbslam::VisualOdometry::Ptr vo ( new acrbslam::VisualOdometry );

    acrbslam::Camera::Ptr camera ( new acrbslam::Camera );

    acrbslam::Data data;    //数据存储类

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

        acrbslam::Frame::Ptr pFrame = acrbslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;

        //数据集时所用时间戳
        pFrame->time_stamp_ = rgb_times[i];
        //
        ACRB_WIFI_DATA.rgb_mat=color;
        ACRB_WIFI_DATA.depth_mat=depth;
        cv::imshow ( "image",  ACRB_WIFI_DATA.rgb_mat);
        cv::waitKey ( 0);
        //

        boost::timer timer;
        vo->addFrame ( pFrame );

        Converter converter;
        converter.se32char(pFrame->T_c_w_, &data.rotation_char, &data.translation_char);
        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == acrbslam::VisualOdometry::LOST )
            break;
    }

}


void* wifi_thread(void *arg)
{   
    wifi_comu wifi_comu_;
    wifi_comu_.wifi_init_uav();

    while(1)
    {   
       // wifi_comu_.rgbmat2rgbchar(wifi_comu_,ACRB_WIFI_DATA.rgb_mat, &ACRB_WIFI_DATA.red, &ACRB_WIFI_DATA.green, &ACRB_WIFI_DATA.blue);
        //wifi_comu_.mat2char(wifi_comu_,ACRB_WIFI_DATA.depth_mat, &ACRB_WIFI_DATA.depth);    //test no depth
//int cin_;
//cin>>cin_;
//if(cin_==1)
//{
    /*
        wifi_comu_.send_data(ACRB_WIFI_DATA.red,3000);
        wifi_comu_.send_data(ACRB_WIFI_DATA.green,307200);
        wifi_comu_.send_data(ACRB_WIFI_DATA.blue,307200);
        wifi_comu_.send_data(ACRB_WIFI_DATA.depth,307200);
    */
        Mat grayframe;
        cvtColor(ACRB_WIFI_DATA.rgb_mat, grayframe, CV_BGR2GRAY);
        wifi_comu_.send_data_new(grayframe);
//}


        
    }
}



}   //namespace acrbslam






int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }
    acrbslam::Config::setParameterFile ( argv[1] );     
    

    pthread_t   thread_wifi;        void *retval_wifi;
    pthread_t   thread_vo;          void *retval_vo;

    int ret_wifi=pthread_create(&thread_wifi,NULL, acrbslam::wifi_thread, NULL);
    int ret_vo=pthread_create(&thread_vo,NULL,acrbslam::vo_thread,NULL);

    while(1);
    pthread_join(thread_wifi,&retval_wifi);
    pthread_join(thread_vo,&retval_vo);
      

    return 0;
}
