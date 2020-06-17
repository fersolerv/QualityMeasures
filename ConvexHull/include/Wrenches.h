#ifndef WRENCHES_H
#define WRENCHES_H

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/thread/thread.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <vector>
#include <math.h>
//#include "kauthamshell.h"
#include "ContactConeGenerator.h"
#include "ConvexHull.h"

using namespace std;
//using namespace Quality


class wrench{
public:
    wrench();
    ~wrench();

    float pquality;
    bool computeWrenchQuality(pcl::PointCloud<pcl::PointXYZ>::Ptr Original, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object2, 
                              pcl::PointCloud<pcl::Normal>::Ptr Normals, 
                              pcl::PointCloud<pcl::Normal>::Ptr Normals2, 
                              Eigen::Vector3f CM);
};



#endif

