#ifndef CONTACTCONEGENERATOR_H
#define CONTACTCONEGENERATOR_H

#include <Eigen/Core>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <algorithm>
#include <string>
#include <time.h>
#include <vector>
#include <cmath> //Para lenguaje C++
#include "Mtools.h"


class CCone
{
    private:

        double unitforce;
        double frictioncoef;
        double frictionAngle;
        double frictionConeRaid;
        double frictionConeHeigh;
        std::vector<Eigen::Vector3f > frictionRimPoints;
        int conesides;

    public:
    
        CCone(float uForce, float fCoef, int cSides);
        ~CCone();

        void frictionCones(Mtools::ContactPoint p, std::vector<Mtools::ContactPoint> &StoreCones);
        void frictionCones(Mtools::ContactPoint p, std::vector<Mtools::ContactPoint> &StoreCones, unsigned int id);
        void frictionCones_2(Mtools::ContactPoint p, std::vector<Mtools::ContactPoint> &StoreCones, unsigned int id);

};

#endif

