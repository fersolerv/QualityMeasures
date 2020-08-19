#include "MVBB.h"
#include "Wrenches.h"
#include <chrono>

using namespace std;
using namespace pcl::console;
using namespace std::chrono;

int main(int argc, char **argv) 
{
    MVBB *qtl; qtl = new MVBB();
    wrench *wrc; wrc = new wrench();

    pcl::PointCloud<pcl::PointXYZ>::Ptr objectPCFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr partialObjectPC(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr objectNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr partialObjectNormals(new pcl::PointCloud<pcl::Normal>);
    Eigen::Vector3f CM;

    if(find_switch(argc, argv, "--computeQTM") && argc == 8)
    {
        auto t1 = high_resolution_clock::now();
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
                              objectPCFiltered, 
                              partialObjectPC, 
                              objectNormals, 
                              partialObjectNormals, 
                              CM))      
            PCL_ERROR("The qualities can't be computed\n");  

        if(!wrc->computeOWSQuality(objectPCFiltered, objectNormals, partialObjectPC, partialObjectNormals, CM))
            PCL_ERROR("OWS can't be computed.\n");
        
        auto t2 = high_resolution_clock::now();
        auto duration = duration_cast<seconds>(t2 - t1).count();
        cout << "Time to compute quality is " << duration << " seconds.\n";
        return 0;
    }
    else if(find_switch(argc, argv, "--computeQTM") && argc != 8)
    {
        PCL_ERROR("Write the command line correctly to compute qualities\n\n");
        qtl->showHelpQuality();
        return -1;
    }
    
    if(find_switch(argc, argv, "--extractValues") && argc == 10) 
    {
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
        if(qtl->getData(inXML, outTransformationTXT, outQualityGraspTXT, qualitySortedTXT))
            cout << "Data extracted\n";
        
        auto t2 = high_resolution_clock::now();
        auto duration = duration_cast<seconds>(t2 - t1).count();
        cout << "Time to extract values is " << duration << " seconds.\n";

        return 0;
    } 
    else if(find_switch(argc, argv, "--extractValues") && argc != 10)
    {
        PCL_ERROR("Write the command line correctly to extract values\n\n");
        qtl->showHelpExtractValues();
        return -1;
    }

    else
        PCL_ERROR("Can't do anything. Please review the possible errors.\n");
        return -1;
}