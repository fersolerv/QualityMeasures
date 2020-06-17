#include "MVBB.h"
#include "Wrenches.h"

using namespace std;
using namespace pcl::console;

int main(int argc, char **argv) {

    MVBB *qtl; qtl = new MVBB();
    wrench *wrc; wrc = new wrench();

    if(find_switch(argc, argv, "--computeQTM")) {
        string graspPointCloud = "";
        string objectPointCloud = "";
        pcl::PointCloud<pcl::PointXYZ>::Ptr Original_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr object_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr object_normals_out(new pcl::PointCloud<pcl::Normal>);
        Eigen::Vector3f CM;

        int index = find_argument(argc, argv, "-graspPointCloud");
        if(index > 0) 
            graspPointCloud = argv[index + 1];
        index = find_argument(argc, argv, "-objectPointCloud");
        if(index > 0) 
            objectPointCloud = argv[index + 1];

        if(!qtl->compute_bbox(graspPointCloud,
                              objectPointCloud, 
                              Original_filtered, 
                              Cloud_out, 
                              object_normals, 
                              object_normals_out, 
                              CM))
            cout << "Cannot perform the computation" << endl;
    }
    else PCL_ERROR("Write the command line correctly\n");
}