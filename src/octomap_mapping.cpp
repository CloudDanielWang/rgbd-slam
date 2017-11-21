#include "acrbslam/octomap_mapping.h"


namespace acrbslam
{


octomap::ColorOcTree *PointCloudMap2Octomap(pointCloud::Ptr CloudMap,Data data, octomap::ColorOcTree *tree)
{
    //cout<<"正在将图像转换为 Octomap ..."<<endl;
    data.SE32Eigen();
                      
    octomap::Pointcloud cloud_octo;
    for (auto p:CloudMap->points)
            cloud_octo.push_back( p.x, p.y, p.z );
        
    tree->insertPointCloud( cloud_octo, octomap::point3d( data.EigenTranslationEstimate(0), data.EigenTranslationEstimate(1), data.EigenTranslationEstimate(2) ) );

    for (auto p:CloudMap->points)
            tree->integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );   

    return tree;
}
    

}//end acrbslam

    


