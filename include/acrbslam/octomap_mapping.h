#ifndef OCTOMAP_H
#define OCTOMAP_H

#include <boost/format.hpp>  // for formating strings

#include <octomap/octomap.h>    // for octomap 
#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>

#include "acrbslam/common_include.h"

#include "acrbslam/camera.h"
#include "acrbslam/cloudmap.h"
#include "acrbslam/data.h"

namespace acrbslam
{
	//void PointCloudMap2Octomap(pointCloud::Ptr CloudMap,Data data);
	//octomap::ColorOcTree PointCloudMap2Octomap(pointCloud::Ptr CloudMap,Data data, octomap::ColorOcTree tree);
	octomap::ColorOcTree *PointCloudMap2Octomap(pointCloud::Ptr CloudMap,Data data, octomap::ColorOcTree *tree);
}

#endif
