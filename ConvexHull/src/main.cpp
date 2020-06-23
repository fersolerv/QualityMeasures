#include "MVBB.h"
#include "Wrenches.h"

using namespace std;
using namespace pcl::console;

int main(int argc, char **argv) {

    MVBB *qtl; qtl = new MVBB();
    wrench *wrc; wrc = new wrench();

    pcl::PointCloud<pcl::PointXYZ>::Ptr originalFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr objectNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr objectNormalsOut(new pcl::PointCloud<pcl::Normal>);
    Eigen::Vector3f CM;

    if(find_switch(argc, argv, "--computeQTM")) {
        string graspPointCloud = "";
        string objectPointCloud = "";
        string transformationsFile = "";
        int index = find_argument(argc, argv, "-graspPointCloud");
        if(index > 0) 
            graspPointCloud = argv[index + 1];
        
        index = find_argument(argc, argv, "-objectPointCloud");
        if(index > 0) 
            objectPointCloud = argv[index + 1];
        
        index = find_argument(argc, argv, "-transformationFile");
        if(index > 0)
            transformationsFile = argv[index + 1];

        if(!qtl->getQualities(graspPointCloud,
                              objectPointCloud,
                              transformationsFile,
                              originalFiltered, 
                              cloudOut, 
                              objectNormals, 
                              objectNormalsOut, 
                              CM))
            cout << "Cannot perform the computation" << endl;
        
        if(!wrc->computeWrenchQuality(originalFiltered, 
                                      cloudOut, 
                                      objectNormals, 
                                      objectNormalsOut,
                                      CM))
            cout << "OWS can't be computed.\n";
    }
    else PCL_ERROR("Write the command line correctly\n");
}