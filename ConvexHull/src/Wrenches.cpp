#include "Wrenches.h"

wrench::wrench(){}

wrench::~wrench(){}

// OWS = Object Wrench Space
float wrench::computeOWS(pcl::PointCloud<pcl::PointXYZ>::Ptr objectPointCloud, 
                        pcl::PointCloud<pcl::Normal>::Ptr objectNormals,  
                        Eigen::Vector3f CM,
                        float uforce,
                        float mu,
                        float csides) {
    
    boost::posix_time::ptime totalst, totalend;
    std::vector <Mtools::ContactPoint> points;
    points.clear();
    Eigen::Vector3f point;
    Eigen::Vector3f norm;
    Mtools::ContactPoint graspPoint;

    CCone *cone; cone = new CCone(uforce, mu, csides);
    ConvexHull *chull; chull = new ConvexHull(CM);

    std::vector <Mtools::ContactPoint> fcones;
    fcones.clear();

    //save points into a variable "contact points"
    for(int i = 0; i < objectPointCloud->points.size(); i ++) {
       point << objectPointCloud->points.at(i).x, objectPointCloud->points.at(i).y, objectPointCloud->points.at(i).z;
       norm << objectNormals->points.at(i).normal[0], objectNormals->points.at(i).normal[1], objectNormals->points.at(i).normal[2];
       graspPoint.p = point;
       graspPoint.n = norm;
       points.push_back(graspPoint);
    }

    //Compute friction cones for every point
    for(int i = 0; i < points.size(); i++)
       cone->frictionCones(points.at(i), fcones, i);
    chull->Cwrenches(fcones); //Compute wrenches
    chull->CreateConvexHull();  //Compute the convex hull of the wrench
    if(chull->isForceClosure()) 
        std::cout<<"Force Closure"<<std::endl;
    else 
        std::cout<<"No force closure"<<std::endl;

    float quality = chull->Quality;
    return quality;
}

bool wrench::computeOWSQuality(pcl::PointCloud<pcl::PointXYZ>::Ptr objectPointCloud, 
                               pcl::PointCloud<pcl::Normal>::Ptr objectNormals, 
                               pcl::PointCloud<pcl::PointXYZ>::Ptr partialObjectPointCloud, 
                               pcl::PointCloud<pcl::Normal>::Ptr partialObjectNormals, 
                               Eigen::Vector3f CM) {

    float totalQuality = this->computeOWS(objectPointCloud, objectNormals, CM, 1.0, 1, 6);
    float potentialQuality = this->computeOWS(partialObjectPointCloud, partialObjectNormals, CM, 1.0, 0.4, 6);
    float LostQualityOWS = (totalQuality - potentialQuality) / totalQuality;
    float QualityOWS = 1 - ((totalQuality - potentialQuality) / totalQuality);
    cout << "Total quality is: " << totalQuality << endl;
    cout << "Potential quality is: " << potentialQuality << endl; 
    cout << "Quality lost based on OWS is: " << LostQualityOWS << endl;
    cout << "\033[1;36mQTows is: " << QualityOWS << "\033[0m" << endl;
    return true;
}