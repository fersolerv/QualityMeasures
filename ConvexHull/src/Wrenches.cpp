#include "Wrenches.h"

wrench::wrench(){}

wrench::~wrench(){}

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

    CCone *cone;
    cone = new CCone(uforce, mu, csides);

    ConvexHull *chull;
    chull = new ConvexHull(CM);

    std::vector <Mtools::ContactPoint> fcones;
    fcones.clear();

    //save points into a variable "contact points"
    for (int i = 0; i < objectPointCloud->points.size(); i ++) {
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
    cout << "Full object quality is: " << quality << endl;

    return quality;
}

bool wrench::computeOWSQuality(pcl::PointCloud<pcl::PointXYZ>::Ptr objectPointCloud, 
                               pcl::PointCloud<pcl::Normal>::Ptr objectNormals, 
                               pcl::PointCloud<pcl::PointXYZ>::Ptr partialObjectPointCloud, 
                               pcl::PointCloud<pcl::Normal>::Ptr partialObjectNormals, 
                               Eigen::Vector3f CM) {

    float totalQuality = computeOWS(objectPointCloud, objectNormals, CM, 1.0, 1, 6);
    float partialQuality = computeOWS(partialObjectPointCloud, partialObjectNormals, CM, 1.0, 0.4, 6);
    float LostQualityOWS = (totalQuality - partialQuality) / totalQuality;
    cout << "Quality lost based on OWS is: " << LostQualityOWS << endl;
    float QualityOWS = 1 - ((totalQuality - partialQuality) / totalQuality);
    cout << "THE QUALITY BASED ON OWS IS: " << QualityOWS << endl;
    return true;
}