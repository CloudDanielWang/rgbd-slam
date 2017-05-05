#include "acrbslam/common_include.h"
#include "acrbslam/camera.h"
#include "acrbslam/cloudmap.h"
#include "acrbslam/frame.h"

#//include <pcl/filters/statistical_outlier_removal.h>      //有毒的头文件。。。

pointCloud::Ptr createPointCloud( acrbslam::Frame::Ptr frame, pointCloud::Ptr orginal_cloud )
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
                 Vector2d pixel( v, u );
                pointWorld= frame->camera_->pixel2world (   pixel, frame->T_c_w_, d);
                
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = frame->color_.data[ v*frame->color_.step+u*frame->color_.channels() ];
                p.g = frame->color_.data[ v*frame->color_.step+u*frame->color_.channels()+1 ];
                p.r = frame->color_.data[ v*frame->color_.step+u*frame->color_.channels()+2 ];
                current->points.push_back( p );
        }
  /*              // depth filter and statistical removal 
        pointCloud::Ptr tmp ( new pointCloud );
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter( *tmp );
        (*orginal_cloud) += *tmp;
*/
            // voxel filter 
    pcl::VoxelGrid<PointT> voxel_filter; 
    voxel_filter.setLeafSize( 0.01, 0.01, 0.01 );       // resolution 
    pointCloud::Ptr tmp ( new pointCloud );
    voxel_filter.setInputCloud( current );
    voxel_filter.filter( *tmp );
    tmp->swap( *current );
    (*orginal_cloud) += *current;


return orginal_cloud;

}
//}
 