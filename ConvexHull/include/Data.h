#ifndef DATA_H
#define DATA_H

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "../libs/pugixml/src/pugixml.hpp"
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;

class Data
{

private:
    bool extractTransforms(const char *inXML, const char *outTransformationTXT);
    bool extractGraspQuality(const char *inXML, const char *outQualityGraspTXT);
    bool qualitySort(const char *inXML, const char *qualitySortedTXT);
    Eigen::Matrix4f returnTransformation(string transformationFilePath, uint line);

public:
    Data();
    ~Data();

    void showHelpQuality();
    void showHelpExtractValues();
    bool getData(const char *inXML, 
                 const char *outTransformationTXT, 
                 const char *outQualityGraspTXT, 
                 const char *qualitySortedTXT);
    string changeGraspNumber(string graspPointCloud, int graspNumber);
    int extractGraspNumber(string graspPointCloudPath);

};
#endif