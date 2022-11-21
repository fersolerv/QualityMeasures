#ifndef WRENCHES_H
#define WRENCHES_H

#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <vector>
#include <math.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "ContactConeGenerator.h"
#include "ConvexHull.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>

using namespace std;

class wrench{

private:

    float computeOWS(pcl::PointCloud<pcl::PointXYZ>::Ptr objectPointCloud, 
                     pcl::PointCloud<pcl::Normal>::Ptr objectNormals,  
                     Eigen::Vector3f CM,
                     float uforce,
                     float mu,
                     float csides);

public:
    wrench();
    ~wrench();

    float pquality;
    bool computeOWSQuality(pcl::PointCloud<pcl::PointXYZ>::Ptr objectPointCloud, 
                           pcl::PointCloud<pcl::Normal>::Ptr objectNormals, 
                           pcl::PointCloud<pcl::PointXYZ>::Ptr partialObjectPointCloud, 
                           pcl::PointCloud<pcl::Normal>::Ptr partialObjectNormals, 
                           Eigen::Vector3f CM);
};
#endif

