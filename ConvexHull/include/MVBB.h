#ifndef MVBB_H
#define MVBB_H

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <vector>
#include <math.h>
#include "MVBB.h"
#include "Wrenches.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
//#include <tf/transform_datatypes.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <pcl/surface/gp3.h>
#include "pugixml.hpp"


class MVBB
{

private:
    std::string GraspCloudPath;
    std::string ObjectCloudPath;
    std::string GraspTransform;

    bool read_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &C_Object, 
                     pcl::PointCloud<pcl::Normal>::Ptr &Normals);

    void cloud_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr Original, 
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &Filtered);

    void getGraspQuality(const char *absPath1, 
                         const char *absPath3);

    void getTransforms(const char *absPath, 
                       const char *absPath2);

    void QualitySort(const char *absPath4, 
                     const char *absPath5);

    void compute_BBox(pcl::PointCloud<pcl::PointXYZ>::Ptr &Hand_configuration, 
                      Eigen::Quaternionf &BBox_Rotation,
                      Eigen::Vector3f &BBox_Traslation, 
                      Eigen::Vector4f &Min, 
                      Eigen::Vector4f &Max, 
                      Eigen::Matrix4f &Projection);

    void ComputeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr C_Object, 
                        pcl::PointCloud<pcl::Normal>::Ptr &Normals, 
                        Eigen::Vector3f &CM);

    void Crop_filters(pcl::PointCloud<pcl::PointXYZ>::Ptr C_Object, 
                      pcl::PointCloud<pcl::Normal>::Ptr Normals, 
                      Eigen::Vector4f Min, 
                      Eigen::Vector4f Max, 
                      Eigen::Matrix4f Projection,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &Points_out, 
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &Points_in, 
                      pcl::PointCloud<pcl::Normal>::Ptr &Normals_ou);

    void Visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr Hand_configuration, 
                   pcl::PointCloud<pcl::PointXYZ>::Ptr Points_out, 
                   Eigen::Vector3f Ctr,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr Points_in, 
                   Eigen::Vector4f Min, Eigen::Vector4f Max, 
                   Eigen::Quaternionf BBox_Rotation, 
                   Eigen::Vector3f BBox_Translation,
                   bool f_cordinates);

    void ModelConstruct(pcl::PointCloud<pcl::PointXYZ>::Ptr C_Object, 
                        double &object_area);

    void ModelConstruct2(pcl::PointCloud<pcl::PointXYZ>::Ptr Points_out, 
                         double &object_area);

public:
    MVBB(int argc, char **argv);
    ~MVBB();

    bool compute_bbox(string graspPointCloudPath,
                      string objectPointCloudPath,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &Original_filtered, 
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &Cloud_out, 
                      pcl::PointCloud<pcl::Normal>::Ptr &object_normals,
                      pcl::PointCloud<pcl::Normal>::Ptr &object_normals_out, 
                      Eigen::Vector3f &CM);

};
#endif

