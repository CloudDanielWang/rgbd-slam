#include "acrbslam/common_include.h"
#include "acrbslam/camera.h"
#include "acrbslam/cloudmap.h"
#include "acrbslam/frame.h"

//#include <pcl/filters/statistical_outlier_removal.h>

pointCloud::Ptr createPointCloud( acrbslam::Frame::Ptr frame ,pointCloud::Ptr orginal_cloud)
{
    pointCloud::Ptr current( new pointCloud );

    // 遍历深度图
    for (int v = 0; v < frame->depth_.rows; v++)
        for (int u=0; u < frame->depth_.cols; u++)
        {
                unsigned int d =frame->depth_.ptr<unsigned short> (v)[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                if ( d >= 7000 ) continue; // 深度太大时不稳定，去掉
                
                Vector3d pointWorld ;
                Vector2d pixel( u, v );
                pointWorld= frame->camera_->pixel2world (   pixel, frame->T_c_w_, d);
                

                PointT p ;
                p.x = -pointWorld[0];
                p.y = -pointWorld[1];
                p.z = pointWorld[2];
                p.b = frame->color_.data[ v*frame->color_.step+u*frame->color_.channels() ];
                p.g = frame->color_.data[ v*frame->color_.step+u*frame->color_.channels()+1 ];
                p.r = frame->color_.data[ v*frame->color_.step+u*frame->color_.channels()+2 ];
                current->points.push_back( p );
        }
    current->height = 1;
    current->width = current->points.size();
     current->is_dense = false;

      cout<<"current cloud size = "<<current->points.size()<<endl;

        // depth filter and statistical removal 
     /*
        pointCloud::Ptr tmp1 ( new pointCloud );
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);	
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter( *tmp1 );
        (*orginal_cloud) += *tmp1;
*/

    // voxel filter 
    static pcl::VoxelGrid<PointT> voxel_filter; 
    voxel_filter.setLeafSize( 0.01, 0.01, 0.01 );       // resolution 
    pointCloud::Ptr tmp ( new pointCloud );
    voxel_filter.setInputCloud( current );
    voxel_filter.filter( *tmp );
    tmp->swap( *current );
    (*orginal_cloud) += *current;
     
    //current->height = 1;
   // current->width = current->points.size();
    cout<<"point cloud size = "<<orginal_cloud->points.size()<<endl;
  
return orginal_cloud;

}

/*
// joinPointCloud 
// 输入：原始点云，新来的帧以及它的位姿
// 输出：将新来帧加到原始帧后的图像
pointcloud::Ptr joinPointCloud( pointcloud::Ptr original, acrbslam::FRAME& newFrame, Eigen::Isometry3d T ) 
{
    pointcloud::Ptr newCloud = createPointCloud( newFrame );

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
 */