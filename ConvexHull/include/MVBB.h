#ifndef MVBB_H
#define MVBB_H

#include <iostream>
#include <string>
#include <limits>
#include <locale> 
#include <algorithm>
#include <iterator>
#include <vector>
#include <math.h>
#include "Wrenches.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
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
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <pcl/surface/gp3.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
class MVBB
{

private:
    bool loadPointCloud(string path, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);
    bool readPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &C_Object, pcl::PointCloud<pcl::Normal>::Ptr &normals);
    void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr original, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered);
    int extractGraspNumber(string graspPointCloudPath);
    Eigen::Matrix4f returnTransformation(string transformationFilePath, uint line);
    void getHandPCTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &handConfiguration, 
                                 Eigen::Quaternionf &bboxRotation,
                                 Eigen::Vector3f &bboxTraslation, 
                                 Eigen::Vector4f &bin, 
                                 Eigen::Vector4f &bax, 
                                 Eigen::Matrix4f &projection,
                                 Eigen::Matrix4f transform);
    void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr C_Object, 
                        pcl::PointCloud<pcl::Normal>::Ptr &Normals, 
                        Eigen::Vector3f &CM);
    float computeQTMpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr C_Object, 
                           pcl::PointCloud<pcl::Normal>::Ptr Normals, 
                           Eigen::Vector4f min, 
                           Eigen::Vector4f max, 
                           Eigen::Matrix4f projection,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsOut, 
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsIn, 
                           pcl::PointCloud<pcl::Normal>::Ptr &NormalsOut);
    void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr handConfiguration, 
                   pcl::PointCloud<pcl::PointXYZ>::Ptr pointsOut, 
                   Eigen::Vector3f centroid,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr pointsIn, 
                   Eigen::Vector4f min, 
                   Eigen::Vector4f max, 
                   Eigen::Quaternionf bboxRotation, 
                   Eigen::Vector3f bboxTranslation,
                   bool fCordinates); 
    float getPointCloudArea(pcl::PointCloud<pcl::PointXYZ>::Ptr C_Object);
    
public:
    MVBB();
    ~MVBB();
    
    bool getQualities(string graspPointCloudPath,
                      string objectPointCloudPath,
                      string transformationsFile,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &originalFiltered, 
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudOut, 
                      pcl::PointCloud<pcl::Normal>::Ptr &objectNormals,
                      pcl::PointCloud<pcl::Normal>::Ptr &objectNormalsOut, 
                      Eigen::Vector3f &CM);
    bool computeQualities(std::string graspPointCloudPath,
                          std::string objectPointCloudPath,
                          std::string transformationsFilePath,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &objectPCFiltered, 
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &partialObjectPC,
                          pcl::PointCloud<pcl::Normal>::Ptr &objectNormals, 
                          pcl::PointCloud<pcl::Normal>::Ptr &partialObjectNormals, 
                          Eigen::Vector3f &CM);
};
#endif