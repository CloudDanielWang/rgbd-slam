/*************************************************************************
	> File Name: src/slamBase.cpp
	> Author: 王征宇
	> Implementation of slamBase.h
	> Created Time: 2017年 月 日
 ************************************************************************/

#include "slamBase.h"
#include<vector>
//*****************************开始对进行点云运算********************************************/
//参考自http://www.cnblogs.com/gaoxiang12/p/4652478.html，修改了三维坐标来源			
pointcloud::Ptr createPointCloud( cv::Mat& img,cv::Mat& cloudMap, cv::Mat& depth )
{
	// 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    pointcloud::Ptr cloud ( new pointcloud );
    // 遍历深度图
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            PointT p;

            // 计算这个点的空间坐标
		p.x =double(cloudMap.at<Vec3f>(m,n)[0]);
		p.y =double(cloudMap.at<Vec3f>(m,n)[1]);
		p.z =double(cloudMap.at<Vec3f>(m,n)[2]);
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = img.ptr<uchar>(m)[n*3];
            p.g = img.ptr<uchar>(m)[n*3+1];
            p.r = img.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    //cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;

return cloud;

}

//*****************************点云保存成功**********************************************/


cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p; // 3D 点
    p.z = double( point.z ); 
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}


// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp( FRAME& frame )
{
	double begin=0;
	double end=0;
	begin=clock();
    	
    	cv::Ptr<Feature2D> orb;
	orb = ORB::create();
            // We can detect keypoint with detect method
	orb->detect(frame.rgb, frame.kp, Mat());
            // and compute their descriptors with method  compute
	orb->compute(frame.rgb, frame.kp, frame.desp);
            // or detect and compute descriptors in one step
	end=clock();
	cout<<"ORB detect time\t"<<(end-begin)/1000.0<<endl;
    return;
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera )
{        

	double begin=0;
	double end=0;
	begin=clock();


	static ParameterReader pd;
	RESULT_OF_PNP result;
	
	vector<String> typeAlgoMatch;
	typeAlgoMatch.push_back("BruteForce-Hamming(2)");   //经初步测试，该算法得出的匹配点较少而且准确        

	vector<DMatch> matches;				//将匹配点放于此，数据为keypoint中的索引
        	cv::Ptr<DescriptorMatcher> descriptorMatcher;

	vector<String>::iterator itMatcher =typeAlgoMatch.begin();
	 
	descriptorMatcher = DescriptorMatcher::create(*itMatcher);
	descriptorMatcher->match(frame1.desp, frame2.desp, matches, Mat());

	end=clock();
	cout<<"ORB match time\t"<<(end-begin)/1000.0<<endl;
	 // Keep best matches only to have a nice drawing.
	 // We sort distance between descriptor matches
/*	
	Mat index;
	int nbMatch=int(matches.size());
	Mat tab(nbMatch, 1, CV_32F);
	for (int i = 0; i<nbMatch; i++)
	{
		tab.at<float>(i, 0) = matches[i].distance;
	}
	sortIdx(tab, index, SORT_EVERY_COLUMN + SORT_ASCENDING);
	vector <DMatch> bestMatches;								//将最佳匹配点存放于bestMatches中
	for (int i = 0; i<100; i++)
	{
	bestMatches.push_back(matches[index.at<int>(i, 0)]);
	}
	Mat matchresult;
*/

	vector <DMatch> bestMatches;								//将最佳匹配点存放于bestMatches中
	double minDis = 9999;
	double good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );
	for ( size_t i=0; i<matches.size(); i++ )
	{
		if ( matches[i].distance < minDis )
		minDis = matches[i].distance;
	}

	if ( minDis < 10 ) 
		minDis = 10;
    
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis)
            bestMatches.push_back( matches[i] );
    }


    if (bestMatches.size() <= 5) 
    {
        return result;
    }


bool openORBwindow = pd.getData("openORBwindow")==string("yes");
if(openORBwindow)
{
	Mat matchresult;
	cv::drawMatches(frame1.rgb, frame1.kp, frame2.rgb, frame2.kp, bestMatches, matchresult);
	namedWindow("ORB匹配结果", WINDOW_AUTOSIZE);
	imshow("ORB匹配结果", matchresult);
	//cout<<"ORB匹配完成\n";
	cout<<"ORB匹配点数\t"<<matches.size()<<endl;
	cout<<"ORBbest匹配点数\t"<<bestMatches.size()<<endl;

	 cv::waitKey(0);
}

///////////////////////////////////////////////////////////////////////////////////////////////
  
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;
	int sum_points=bestMatches.size();

    // 相机内参
    for (int i=0; i<sum_points; i++)
    {
		int ref =bestMatches[i].queryIdx;  
		int now =bestMatches[i].trainIdx;
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[ref].pt;

	 //ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
	double d=double(frame1.cloudMap.at<Vec3f>(frame1.kp[ref].pt.y,  frame1.kp[ref].pt.x)[2]);
	//double d=double(frame1.cloudMap.at<Vec3f>(frame1.kp[bestMatches[i].trainIdx].pt.y,  frame1.kp[bestMatches[i].trainIdx].pt.x)[2]);
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！

       if(d == 0.0) continue;   //若有一点的深度值为0，舍去，进入下一次循环。

        pts_img.push_back( cv::Point2f( frame2.kp[now].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f p3d = point2dTo3d( pt, camera );
        pts_obj.push_back(p3d);
    }

    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        return result;
    }

    double camera_matrix_data[3][3] = 
    {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, SOLVEPNP_P3P );

    result.rvec = rvec;
    result.tvec = tvec;

    //cout<<"R="<<rvec*180.0/3.1415926<<endl;
    //cout<<"traslation="<<tvec*1000.0<<endl;    

    return result;
}


// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ ) 
            r(i,j) = R.at<double>(i,j);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(1,0); 
    T(2,3) = tvec.at<double>(2,0);

	//cout<<"T.matrix()"<<T.matrix()<<endl;
    return T;
}

// joinPointCloud 
// 输入：原始点云，新来的帧以及它的位姿
// 输出：将新来帧加到原始帧后的图像
pointcloud::Ptr joinPointCloud( pointcloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) 
{
    pointcloud::Ptr newCloud = createPointCloud( newFrame.rgb, newFrame.cloudMap, newFrame.depth );

    // 合并点云
    pointcloud::Ptr output (new pointcloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<PointT> voxel;
    static ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newCloud );
    pointcloud::Ptr tmp( new pointcloud() );
    voxel.filter( *tmp );
    return tmp;
}

