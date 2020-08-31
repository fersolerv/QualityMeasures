#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <Eigen/Core>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <algorithm>
#include <string>
#include <time.h>
#include <vector>
#include <cmath> 
#include "Mtools.h"

extern "C" {
#include "libqhull/qhull_a.h"
}

class ConvexHull {
private:

    bool convertPoints(double* storePointsQHull);
    float minOffset();
    void ChullCenter();

public:
    
    std::vector<Mtools::ContactPoint> Wrenches;
    std::vector<std::vector<Eigen::VectorXf> > EWR;
    Mtools::ConvexHUll6D Result;
    Eigen::Vector3f CM;

    float Quality;
    float mOffset;
    Mtools::ContactPoint Center;

    ConvexHull();
    ConvexHull(Eigen::Vector3f centerofmodel);
    ~ConvexHull();
    bool CreateConvexHull();
    void Cwrenches(std::vector<Mtools::ContactPoint>& cpoints, int sides);
    void Cwrenches(std::vector<Mtools::ContactPoint>& cpoints);
    bool isForceClosure();
    float minDistCH();
};

#endif

