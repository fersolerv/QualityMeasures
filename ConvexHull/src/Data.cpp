#include "Data.h"
#include <boost/algorithm/string/replace.hpp>

using namespace std;

Data::Data() {}
Data::~Data(){};

void Data::showHelpQuality() {
    cout << "Usage: ./TransferQualityMeasure --computeQTM -graspPointCloud [path + filename]  -objectPointCloud [path + filename] -transformationFile [path + file]\n" << endl;
    cout << "    TransferQualityMeasure:        Executable file." << endl;
    cout << "    --computeQTM:                  Method to use." << endl;
    cout << "    -graspPointCloud:              Path where the grasp point cloud is. " << endl;
    cout << "    -objectPointCloud:             Path where the object point cloud is. " << endl;
    cout << "    -transformationFile:           Path where the transformations values are.\n" << endl;
}

void Data::showHelpExtractValues() {
    cout << "Usage: ./TransferQualityMeasure --extractValues -transformationXMLFile [path + filename]  -outputGraspTransformationPath [path + filename] -outputGraspQualityPath [path + file] -outputSortedQualitiesPath [path + file]\n" << endl;
    cout << "    TransferQualityMeasure:              Executable file." << endl;
    cout << "    --extractValues:                     Method to use." << endl;
    cout << "    -transformationXMLFile:              Path where the XML file is. " << endl;
    cout << "    -outputGraspTransformationPath:      Path to put transformation values from file. " << endl;
    cout << "    -outputGraspQualityPath:             Path to put the qualities values." << endl;
    cout << "    -outputSortedQualitiesPath:          Path to put the sorted qualities values.\n" << endl;
}

bool Data::extractTransforms(const char *inXML, const char *outTransformationTXT) {   
    //get the matrix transformation of each grasp from the .xml file and save it into a .txt file
    ofstream transform(outTransformationTXT);
    double xx, xy, xz, yx, yy, yz, zx, zy, zz, x1, y1, z1, p1, p2, p3, p4;
    pugi::xml_document doc;
    //Load .xml file
    pugi::xml_parse_result result = doc.load_file(inXML);
    if (!result) 
        std::cout << "Parse error: " << result.description() << ", character pos = " << result.offset<<std::endl;
    else 
        std::cout << "Problem file loaded"<<std::endl;
    
    int i = 0;
    for(pugi::xml_node tool = doc.child("ManipulationObject").child("GraspSet").child("Grasp"); tool; tool = tool.next_sibling("Grasp")) {
        
        pugi::xml_node node = tool.child("Transform").child("Matrix4x4").child("row1");
        xx = node.attribute("c1").as_double();
        xy = node.attribute("c2").as_double();
        xz = node.attribute("c3").as_double();
        x1 = node.attribute("c4").as_double();

        node = tool.child("Transform").child("Matrix4x4").child("row2");

        yx = node.attribute("c1").as_double();
        yy = node.attribute("c2").as_double();
        yz = node.attribute("c3").as_double();
        y1 = node.attribute("c4").as_double();

        node = tool.child("Transform").child("Matrix4x4").child("row3");

        zx = node.attribute("c1").as_double();
        zy = node.attribute("c2").as_double();
        zz = node.attribute("c3").as_double();
        z1 = node.attribute("c4").as_double();

        node = tool.child("Transform").child("Matrix4x4").child("row4");

        p1 = node.attribute("c1").as_double();
        p2 = node.attribute("c2").as_double();
        p3 = node.attribute("c3").as_double();
        p4 = node.attribute("c4").as_double();

        transform << xx << ", ";
        transform << xy << ", ";
        transform << xz << ", ";
        transform << x1 << ", ";

        transform << yx << ", ";
        transform << yy << ", ";
        transform << yz << ", ";
        transform << y1 << ", ";

        transform << zx << ", ";
        transform << zy << ", ";
        transform << zz << ", ";
        transform << z1 << ", ";

        transform << p1 << ", ";
        transform << p2 << ", ";
        transform << p3 << ", ";
        transform << p4 << ";" << endl;

        i++;
    }
    return true;
}

bool Data::extractGraspQuality(const char *inXML, const char *outQualityGraspTXT) { 
    //get the quality of each grasp from the .xml file and save it into a .txt file
    ofstream graspQuality(outQualityGraspTXT);
    double quality;
    pugi::xml_document doc;
    //Load .xml file
    pugi::xml_parse_result result = doc.load_file(inXML);
    if (!result) 
        cout << "Parse error: " << result.description() << ", character pos = " << result.offset << endl;
    else 
        cout << "Problem file loaded" << endl;
    
    int i = 0;
    for(pugi::xml_node tool = doc.child("ManipulationObject").child("GraspSet").child("Grasp"); tool; tool = tool.next_sibling("Grasp")) {
        quality = tool.attribute("quality").as_double();
        graspQuality << quality << endl;
        i++;
    }
    return true;
}

bool Data::qualitySort(const char *inXML, const char *qualitySortedTXT) {          
    //Extract qualities
    ofstream graspQuality(qualitySortedTXT);
    double quality;
    pugi::xml_document doc;
    //Load .xml file
    pugi::xml_parse_result result = doc.load_file(inXML);
    if (!result)
        cout << "Parse error: " << result.description()<< ", character pos = " << result.offset << endl;
    else
        cout << "Problem file loaded" << endl;

    for(pugi::xml_node tool = doc.child("ManipulationObject").child("GraspSet").child("Grasp"); tool; tool = tool.next_sibling("Grasp")) {
        int i = 0;
        quality = tool.attribute("quality").as_double();
        graspQuality << quality << endl;
        i++;
    }
    // Sort qualities
    ifstream file(qualitySortedTXT);
    vector<string> rows;

    // Read all the lines and add them to the rows vector
    while(!file.eof()) {
        string line;
        getline(file, line);
        rows.push_back(line);
    }

    sort(rows.begin(), rows.end());
    return true;
}

int Data::extractGraspNumber(string graspPointCloudPath) {
    // For atoi, the input string has to start with a digit, so lets search for the first digit
    size_t i = 0;
    for ( ; i < graspPointCloudPath.length(); i++ ) { 
        if (isdigit(graspPointCloudPath[i])) 
            break; 
    }
    // remove the first chars, which aren't digits
    graspPointCloudPath = graspPointCloudPath.substr(i, graspPointCloudPath.length() - i );
    int number = atoi(graspPointCloudPath.c_str());
    return number;
}

string Data::changeGraspNumber(string graspPointCloudPath, int graspNumber) {
    
    // Convert grasp number in string
    string graspStr = std::to_string(graspNumber);

    // Find digit in string
    string graspPCString;
    string newGraspPointCloudPath;
    size_t i = 0;
    for ( ; i < graspPointCloudPath.length(); i++ ) {
        if (isdigit(graspPointCloudPath[i])) {
            break;
        }
    }
    graspPCString = graspPointCloudPath.substr(i, graspPointCloudPath.length() - i );
    int number = atoi(graspPCString.c_str());
    string num = std::to_string(number);

    // in place
    newGraspPointCloudPath = graspPointCloudPath;
    boost::replace_all(newGraspPointCloudPath, num, graspStr);

    return newGraspPointCloudPath;
}

bool Data::getData(const char *inXML, 
                   const char *outTransformationTXT, 
                   const char *outQualityGraspTXT, 
                   const char *qualitySortedTXT) {  


    extractTransforms(inXML, outTransformationTXT);
    extractGraspQuality(inXML, outQualityGraspTXT);
    qualitySort(inXML, qualitySortedTXT);
    return true;
}

Eigen::Matrix4f Data::returnTransformation(string transformationFilePath, uint line) {
    string sLine = "";
    ifstream read;
    read.open(transformationFilePath);
    int line_no = 0;
    while (line_no != line && getline(read, sLine))
        ++line_no;
    
    Eigen::Matrix4f transformation;
    transformation = Eigen::Matrix4f::Identity();
    std::vector<float> num;
    std::stringstream ss(sLine);

    float i;
    while (ss >> i) {
        num.push_back(i);
        if (ss.peek() == ',')
            ss.ignore();
    }
    
    transformation << num[0], num[1], num[2], num[3], 
                      num[4], num[5], num[6], num[7], 
                      num[8], num[9], num[10], num[11], 
                      0, 0, 0, 1;
    read.close();
    return transformation;
}