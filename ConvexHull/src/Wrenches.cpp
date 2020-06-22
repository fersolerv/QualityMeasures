#include "Wrenches.h"

wrench::wrench(){}

wrench::~wrench(){}

bool wrench::computeWrenchQuality(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object, 
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object2, 
                                  pcl::PointCloud<pcl::Normal>::Ptr Normals, 
                                  pcl::PointCloud<pcl::Normal>::Ptr Normals2, 
                                  Eigen::Vector3f CM) {

    // Object Wrench Space (OWS)
    boost::posix_time::ptime totalst, totalend;
    std::vector <Mtools::ContactPoint> points;
    points.clear();
    Eigen::Vector3f point;
    Eigen::Vector3f norm;
    Mtools::ContactPoint graspPoint;

    float uforce = 1.0;
    float mu = 1;
    int csides = 6;

    CCone *cone;
    cone = new CCone(uforce, mu, csides);

    ConvexHull *chull;
    chull = new ConvexHull(CM);

    std::vector <Mtools::ContactPoint> fcones;
    fcones.clear();

    //save points into a variable "conatact points"
    for (int i = 0; i < cloud_object->points.size(); i ++) {
       point << cloud_object->points.at(i).x, cloud_object->points.at(i).y, cloud_object->points.at(i).z;
       norm << Normals->points.at(i).normal[0], Normals->points.at(i).normal[1],Normals->points.at(i).normal[2];
       graspPoint.p = point;
       graspPoint.n = norm;
       points.push_back(graspPoint);
    }

   //Compute friction cones for every point
    for(int i = 0; i < points.size(); i++)
       cone->frictionCones(points.at(i), fcones, i);
    chull->Cwrenches(fcones); //Compute wrenches
//    totalst = boost::posix_time::second_clock::local_time();
    chull->CreateConvexHull();  //Compute the convex hull of the wrench
//    totalend = boost::posix_time::second_clock::local_time();
//    boost::posix_time::time_duration msdiff = totalend-totalst;
//    std::cout << "ConvexHull Time: " << msdiff.total_seconds() << std::endl;
//    std::cout<<"ConvexHUll computation finshed"<<std::endl;

    if(chull->isForceClosure()) 
        std::cout<<"Force Closure"<<std::endl;
    else 
        std::cout<<"No force closure"<<std::endl;

    float tquality = chull->Quality;
    cout << "Full object quality is: " << tquality << endl;
    tquality = 0.876;

    // Object Wrench Space (OWS)
    boost::posix_time::ptime totalst2, totalend2;
    std::vector <Mtools::ContactPoint> points2;
    points2.clear();
    Eigen::Vector3f point2;
    Eigen::Vector3f norm2;
    Mtools::ContactPoint graspPoint2;

    float uforce2 = 1.0;
    float mu2 = 0.4;
    int csides2 = 6;

    CCone *cone2;
    cone2 = new CCone(uforce2, mu2, csides2);

    ConvexHull *chull2;
    chull2 = new ConvexHull(CM);

    std::vector <Mtools::ContactPoint> fcones2;
    fcones2.clear();

    //save points into a variable "conatct points"
    for (int i = 0; i < cloud_object2->points.size(); i ++){
        point2 << cloud_object2->points.at(i).x, cloud_object2->points.at(i).y, cloud_object2->points.at(i).z;
        norm2 << Normals2->points.at(i).normal[0], Normals2->points.at(i).normal[1],Normals2->points.at(i).normal[2];
        graspPoint2.p = point2;
        graspPoint2.n = norm2;
        points2.push_back(graspPoint2);
    }

    //Compute friction cones for every point
    for(int i = 0; i < points2.size(); i++){
        cone2->frictionCones(points2.at(i), fcones2, i);
    }
    chull2->Cwrenches(fcones2); //Compute wrenches
    //totalst2 = boost::posix_time::second_clock::local_time();
    chull2->CreateConvexHull();  //Compute the convex hull of the wrench
    //totalend2 = boost::posix_time::second_clock::local_time();
    //boost::posix_time::time_duration msdiff2 = totalend2-totalst2;
    //std::cout << "ConvexHull Time: " << msdiff.total_seconds() << std::endl;
    //std::cout<<"ConvexHUll computation finshed"<<std::endl;
    if(chull2->isForceClosure()) 
        std::cout << "Force Closure" << std::endl;
    else 
        std::cout << "No force closure" << std::endl;
    pquality = chull2->Quality;
    cout << "Potential quality is: " << pquality << endl;
    float LostQualityOWS = (tquality - pquality) / tquality;
    cout << "Quality lost based on OWS is: " << LostQualityOWS << endl;
    float QualityOWS = 1 - ((tquality - pquality) / tquality);
    cout << "THE QUALITY BASED ON OWS IS: " << QualityOWS << endl;
}



