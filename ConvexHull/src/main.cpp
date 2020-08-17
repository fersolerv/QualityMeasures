#include "MVBB.h"
#include "Wrenches.h"

using namespace std;
using namespace pcl::console;

int main(int argc, char **argv) 
{
    MVBB *qtl; qtl = new MVBB();
    wrench *wrc; wrc = new wrench();

    pcl::PointCloud<pcl::PointXYZ>::Ptr objectPCFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr partialObjectPC(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr objectNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr partialObjectNormals(new pcl::PointCloud<pcl::Normal>);
    Eigen::Vector3f CM;

    if(find_switch(argc, argv, "--computeQTM")) 
    {
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
            cout << "Cannot perform the computation" << endl;
        
        if(!wrc->computeOWSQuality(objectPCFiltered, objectNormals, partialObjectPC, partialObjectNormals, CM))
            cout << "OWS can't be computed.\n";
    }
    else if(argc != 3)
    {
        PCL_ERROR("Write the command line correctly\n");
        qtl->showHelpQuality();
        return -1;
    }
    
    if(find_switch(argc, argv, "--extractValues")) 
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

        if(qtl->getData(inXML, outTransformationTXT, outQualityGraspTXT, qualitySortedTXT))
            cout << "Data extracted\n";
        
    }
    else if(argc != 3)
    {
        PCL_ERROR("Write the command line correctly\n");
        qtl->showHelpExtractValues();
        return -1;
    }
}