/*************************************************************************
	> File Name: src/slamBase.cpp
	> Author: 王征宇
	> Implementation of slamBase.h
	> Created Time: 2017年0 日
 ************************************************************************/


#include "slamBase.h"
//using namespace cv;
//using namespace pcl;
using namespace std;
    
// 选择优化方法
    typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
    typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 

double normofTransform( cv::Mat rvec, cv::Mat tvec );

/*****************************回环检测部分**********************************/
// 检测两个帧，结果定义
enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME}; 
// 函数声明 关键帧检测
CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops=false );
// 检测近距离的回环
void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );
// 随机检测回环
void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti);
/************************************************************************/

/************************************************************************/
/*                                                 ！！！！主函数！！！！                                                     */
/************************************************************************/

int main()
{

	VideoCapture capture(CV_CAP_OPENNI);    //设置视频的来源为OPENNI设备，即Kinect

	ParameterReader pd;		//读取文件中的参数
	int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
	int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );

	// 所有的关键帧都放在了这里
	vector< FRAME > keyframes; 

	// initialize
	cout<<"Initializing ..."<<endl;
	int currIndex = startIndex; // 当前索引为currIndex
	
	FRAME frame_now;			//第一帧的获取
	capture.grab();       			
	capture.retrieve( frame_now.rgb, CV_CAP_OPENNI_BGR_IMAGE );   	
	capture.retrieve( frame_now.depth, CV_CAP_OPENNI_DEPTH_MAP );   
	capture.retrieve( frame_now.cloudMap,  CV_CAP_OPENNI_POINT_CLOUD_MAP );

	computeKeyPointsAndDesp(frame_now);

	pointcloud:: Ptr cloud_now=createPointCloud( frame_now.rgb, frame_now.cloudMap, frame_now.depth );

	frame_now.frameID=currIndex;
	//frame_now.rotate = Mat::zeros(3,1,CV_32F);
	//frame_now.translation = Mat::zeros(3,1,CV_32F);
	
    /******************************* 
    // 新增:有关g2o的初始化
    *******************************/
    // 初始化求解器
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

    g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
    globalOptimizer.setAlgorithm( solver ); 
    // 不要输出调试信息
    globalOptimizer.setVerbose( false );

    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( currIndex );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );

    keyframes.push_back( frame_now );		//将第一帧的信息放入keyframes

    double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    bool check_loop_closure = pd.getData("check_loop_closure")==string("yes");
    /**********************************************************************************/
	int badframesnum=0;
for(currIndex=startIndex+1; currIndex<endIndex; currIndex++ )
{	
	

	FRAME frame_now;			//必须在循环内进行初始化，否则在第一次循环后，frame_now的值将不正确，导致解算出的R矩阵不变
	capture.grab();       			
	capture.retrieve( frame_now.rgb, CV_CAP_OPENNI_BGR_IMAGE );   	
	capture.retrieve( frame_now.depth, CV_CAP_OPENNI_DEPTH_MAP );   
	capture.retrieve( frame_now.cloudMap,  CV_CAP_OPENNI_POINT_CLOUD_MAP );

	computeKeyPointsAndDesp(frame_now);

	pointcloud:: Ptr cloud_now=createPointCloud( frame_now.rgb, frame_now.cloudMap, frame_now.depth );
	frame_now.frameID=currIndex;
	//frame_now.rotate = Mat::zeros(3,1,CV_32F);
	//frame_now.translation = Mat::zeros(3,1,CV_32F);
/****************************************************************/
        CHECK_RESULT result = checkKeyframes( keyframes.back(), frame_now, globalOptimizer ); //匹配该帧与keyframes里最后一帧
        switch (result) // 根据匹配结果不同采取不同策略
        {
        case NOT_MATCHED:
            //没匹配上，直接跳过
            cout<<RED"Not enough inliers."<<endl;badframesnum++;
            break;
        case TOO_FAR_AWAY:
            // 太远了，可能出错了
            cout<<RED"Too far away, may be an error."<<endl;badframesnum++;
            break;
        case TOO_CLOSE:
            // 太近了，也直接跳
            cout<<RESET"Too close, not a keyframe"<<endl;
            break;
        case KEYFRAME:
            cout<<GREEN"This is a new keyframe"<<endl;
            // 不远不近，刚好						//获取到关键帧之后，开始回环检测
            /**
             * This is important!!
             * This is important!!
             * This is important!!
             * (very important so I've said three times!)
             */

            // 检测回环
            if (check_loop_closure)
            {	int flag =0;
                checkNearbyLoops( keyframes, frame_now, globalOptimizer);		//将当前帧与最近几帧进行比较
                checkRandomLoops( keyframes, frame_now, globalOptimizer);		//将当前帧与之前的帧进行随机比较
            }



            keyframes.push_back( frame_now );
            
            break;
	default:
            break;
        }
 
/****************************************************************/

}
/****************************************************************/
	    // 优化所有边
    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("../data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    globalOptimizer.save( "../data/result_after.g2o" );
    cout<<"Optimization done."<<endl;

    // 拼接点云地图
    cout<<"saving the point cloud map..."<<endl;
    pointcloud::Ptr output ( new pointcloud() ); //全局地图
    pointcloud::Ptr tmp ( new pointcloud() );

    pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
    pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
    pass.setFilterFieldName("z");
    pass.setFilterLimits( 0.0, 5.0 ); //4m以上就不要了

    double gridsize = atof( pd.getData( "voxel_grid" ).c_str() ); //分辨图可以在parameters.txt里调
    voxel.setLeafSize( gridsize, gridsize, gridsize );

    for (size_t i=0; i<keyframes.size(); i++)
    {
        // 从g2o里取出一帧
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
        Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
	pointcloud::Ptr newCloud = createPointCloud( keyframes[i].rgb,keyframes[i].cloudMap ,keyframes[i].depth ); //转成点云
        // 以下是滤波
        voxel.setInputCloud( newCloud );
        voxel.filter( *tmp );
        pass.setInputCloud( tmp );
        pass.filter( *newCloud );
        // 把点云变换后加入全局地图中
        pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
        *output += *tmp;
        tmp->clear();
        newCloud->clear();

	//cout<<"pose"<<pose.matrix()<<endl;
    }

    voxel.setInputCloud( output );
    voxel.filter( *tmp );
    //存储
    pcl::io::savePCDFile( "../data/result.pcd", *tmp );
    
    cout<<"Final map is saved."<<endl;

	//cout<<"Rotate\n"<<keyframes[keyframes.size()-1].rotate*180.0/3.1415926<<endl;
	//cout<<"Translation\n"<<keyframes[keyframes.size()-1].translation*1000.0<<endl;
/****************************************************************/
	//while( !viewer.wasStopped() )
	//{}
    return 0;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops)
{
    static ParameterReader pd;
    static double max_norm = atof( pd.getData("max_norm").c_str() );
    static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
    static CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    // 比较f1 和 f2
    RESULT_OF_PNP result = estimateMotion( f1, f2, camera );

    // 计算运动范围是否太大
    double norm = normofTransform(result.rvec, result.tvec);
	cout<<"norm="<<norm<<endl;
    if ( is_loops == false )
    {
        if ( norm >= max_norm )
            return TOO_FAR_AWAY;   // too far away, may be error
    }
    else
    {
        if ( norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }

    if ( norm <= keyframe_threshold )
        return TOO_CLOSE;   // too adjacent frame
    // 向g2o中增加这个顶点与上一帧联系的边
    // 顶点部分
    // 顶点只需设定id即可
    if (is_loops == false)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( f2.frameID );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex(v);
    }
/*
	f2.rotate.at<double>(0,0)+=result.rvec.at<double>(0,0);
	f2.rotate.at<double>(1,0)+=result.rvec.at<double>(1,0);
	f2.rotate.at<double>(2,0)+=result.rvec.at<double>(2,0);
	f2.translation.at<double>(0,0)+=result.tvec.at<double>(0,0);
	f2.translation.at<double>(1,0)+=result.tvec.at<double>(1,0);
	f2.translation.at<double>(2,0)+=result.tvec.at<double>(2,0);
*/
    // 边部分
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // 连接此边的两个顶点id
    edge->setVertex( 0, opti.vertex(f1.frameID ));
    edge->setVertex( 1, opti.vertex(f2.frameID ));
    edge->setRobustKernel( new g2o::RobustKernelHuber() );
    // 信息矩阵
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    // 也可以将角度设大一些，表示对角度的估计更加准确
    edge->setInformation( information );
    // 边的估计即是pnp求解之结果
    Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
    // edge->setMeasurement( T );
    edge->setMeasurement( T.inverse() );
    // 将此边加入图中
    opti.addEdge(edge);

    return KEYFRAME;
}

void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti)
{
    static ParameterReader pd;
    static int nearby_loops= atoi( pd.getData("nearby_loops").c_str() );

    
    // 就是把currFrame和 frames里末尾几个测一遍
    if ( frames.size() <= nearby_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-nearby_loops; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
}

void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti)
{
    static ParameterReader pd;
    static int random_loops= atoi( pd.getData("random_loops").c_str() );


    srand( (unsigned int) time(NULL) );
    // 随机取一些帧进行检测
    
    if ( frames.size() <= random_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // randomly check loops
        for (int i=0; i<random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes( frames[index], currFrame, opti, true );
        }
    }
}
