#include "../include/MVBB.h"
#include "Wrenches.h"
#include "../include/Data.h"
#include <chrono>

using namespace std;
using namespace pcl::console;
using namespace chrono;

int main(int argc, char **argv) {

    MVBB *qtl; qtl = new MVBB();
    wrench *wrc; wrc = new wrench();
    Data *data; data = new Data();

    pcl::PointCloud<pcl::PointXYZ>::Ptr objectPCFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr partialObjectPC(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr objectNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr partialObjectNormals(new pcl::PointCloud<pcl::Normal>);
    Eigen::Vector3f CM;
    auto t1 = high_resolution_clock::now();

    if(find_switch(argc, argv, "--computeQTM") && argc == 8) {
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
        
        qtl->computeQualities(graspPointCloud,
                              objectPointCloud,
                              transformationsFile,
                              objectPCFiltered, 
                              partialObjectPC, 
                              objectNormals, 
                              partialObjectNormals, 
                              CM);

        // if(!wrc->computeOWSQuality(objectPCFiltered, objectNormals, partialObjectPC, partialObjectNormals, CM)) {
        //     PCL_ERROR("OWS can't be computed.\n");
        //     return -1;
        // }
        
        auto t2 = high_resolution_clock::now();
        auto duration = duration_cast<seconds>(t2 - t1).count();
        cout << "\033[1;32mTime to compute quality is " << duration << " seconds.\033[0m\n";

    }
    else if(find_switch(argc, argv, "--computeQTM") && argc != 8) {
        PCL_ERROR("Write the command line correctly to compute qualities\n\n");
        data->showHelpQuality();
        return -1;
    }
    
    if(find_switch(argc, argv, "--extractValues") && argc == 10) {
        const char *inXML, *outTransformationTXT, *outQualityGraspTXT, *qualitySortedTXT;
   
        int index = find_argument(argc, argv, "-transformationXMLFile"); // file.xml
        if(index > 0)
            inXML = argv[index + 1];

        index = find_argument(argc, argv, "-outputGraspTransformationPath"); // file.txt
        if(index > 0)
            outTransformationTXT = argv[index + 1];
        
        index = find_argument(argc, argv, "-outputGraspQualityPath"); // file.txt
        if(index > 0)
            outQualityGraspTXT = argv[index + 1];
        
        index = find_argument(argc, argv, "-outputSortedQualitiesPath"); // file.txt
        if(index > 0)
            qualitySortedTXT = argv[index + 1];

        auto t1 = high_resolution_clock::now();
        if(data->getData(inXML, outTransformationTXT, outQualityGraspTXT, qualitySortedTXT))
            cout << "Data extracted\n";
        
        auto t2 = high_resolution_clock::now();
        auto duration = duration_cast<seconds>(t2 - t1).count();
        cout << "Time to extract values is " << duration << " seconds.\n";

        return 0;
    } 
    else if(find_switch(argc, argv, "--extractValues") && argc != 10) {
        PCL_ERROR("Write the command line correctly to extract values\n\n");
        data->showHelpExtractValues();
        return -1;
    }

    return 0;
}