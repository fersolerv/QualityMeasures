#include "MVBB.h"
#include "Wrenches.h"


using namespace std;

int main(int argc, char **argv) {

    MVBB *qtl;
    qtl = new MVBB(argc, argv);

    wrench *wrc;
    wrc = new wrench();

    pcl::PointCloud<pcl::PointXYZ>::Ptr Original_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr object_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr object_normals_out(new pcl::PointCloud<pcl::Normal>);
    Eigen::Vector3f CM;

    //Compute Bounding box of the grasp
    // if(!qtl->compute_bbox(Original_filtered, Cloud_out, object_normals, object_normals_out, CM))
    //     cout<<"Imposible perform the computation"<<std::endl;

    //Tranfer Quality based on OWS of the object
   // wrc->computeWrenchQuality(Original_filtered, Cloud_out, object_normals, object_normals_out, CM/*, PointsOut */);
}