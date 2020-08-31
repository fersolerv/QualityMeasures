#include "ContactConeGenerator.h"

CCone::CCone(float uForce, float fCoef, int cSides) {
    //Generation of generic friction cones
    this->unitforce = uForce;
    this->frictioncoef = fCoef;
    this->conesides = cSides;
    this->frictionAngle = atan(frictioncoef);
    this->frictionConeRaid = unitforce * sin(frictionAngle);
    this->frictionConeHeigh = unitforce * cos(frictionAngle);

    for(int i = 0; i < conesides; i++) {
        Eigen::Vector3f p;
        p(0) = unitforce * (float)(cos(frictionAngle) * cos(i * 2.0 * M_PI / conesides));
        p(1) = unitforce * (float)(cos(frictionAngle) * sin(i * 2.0 * M_PI / conesides));
        p(2) = (float)frictionConeHeigh;
        frictionRimPoints.push_back(p);
    }

}

CCone::~CCone() {

    frictionRimPoints.clear();
}

void CCone::frictionCones(Mtools::ContactPoint p, std::vector<Mtools::ContactPoint>& StoreCones) {
    bool printInfo =  false;
    if(printInfo) {
        std::cout<< "Compute Friction Cones" << std::endl;
        std::cout<< "Point: " << std::endl;
        std::cout<< p.p << std::endl;
        std::cout<< "Normal: " << std::endl;
        std::cout<< p.n << std::endl;
    }

    //Rotate Generic friction cone to align  with the object normal
    Eigen::Vector3f upRightNormal(0.0f,0.0f,1.0f);
    Mtools::Quaternion ObNorRot = Mtools::getRot(upRightNormal,p.n);
    Eigen::Matrix4f ObNorT = Mtools::quat2eigen4f(ObNorRot);

    Eigen::Vector3f conePoint;

    for(int i = 0; i < conesides; i++) {
        Mtools::ContactPoint nConePoint;
        Eigen::Vector3f conePointOrg = frictionRimPoints[i];
        conePoint = Mtools::TPosition(conePointOrg, ObNorT);
        nConePoint.p = conePoint + p.p;
        nConePoint.n = conePoint;
        nConePoint.n.normalize();

        if(printInfo) {
            std::cout<< "Loop " << i <<std::endl;
            std::cout<< "nCone: " << std::endl;
            std::cout<< nConePoint.p << std::endl;
            std::cout<<std::endl;
        }
        StoreCones.push_back(nConePoint);
    }

}

void CCone::frictionCones(Mtools::ContactPoint p, 
                          std::vector<Mtools::ContactPoint>& StoreCones, 
                          unsigned int id) {
    bool printInfo =  false;
    if(printInfo) {
        std::cout<< "Compute Friction Cones" << std::endl;
        std::cout<< "Point: " << std::endl;
        std::cout<< p.p << std::endl;
        std::cout<< "Normal: " << std::endl;
        std::cout<< p.n << std::endl;
    }

    //Rotate Generic friction cone to align  with the object normal
    Eigen::Vector3f upRightNormal(0.0f,0.0f,1.0f);
    Mtools::Quaternion ObNorRot = Mtools::getRot(upRightNormal,p.n);
    Eigen::Matrix4f ObNorT = Mtools::quat2eigen4f(ObNorRot);

    Eigen::Vector3f conePoint;
    for(int i = 0; i < conesides; i++) {
        Mtools::ContactPoint nConePoint;
        Eigen::Vector3f conePointOrg = frictionRimPoints[i];
        conePoint = Mtools::TPosition(conePointOrg, ObNorT);
        nConePoint.p = conePoint + p.p;
        nConePoint.n = conePoint;
        nConePoint.n.normalize();
        nConePoint.id = id;

        if(printInfo) {
            std::cout<< "Loop " << i <<std::endl;
            std::cout<< "nCone: " << std::endl;
            std::cout<< nConePoint.p <<" "<<id<< std::endl;
            std::cout<<std::endl;

        }
        StoreCones.push_back(nConePoint);
    }

}

void CCone::frictionCones_2(Mtools::ContactPoint p,
                            std::vector<Mtools::ContactPoint> &StoreCones, 
                            unsigned int id) {

    bool printInfo =  false;

    if(printInfo) {
        std::cout<< "Compute Friction Cones" << std::endl;
        std::cout<< "Point: " << std::endl;
        std::cout<< p.p << std::endl;
        std::cout<< "Normal: " << std::endl;
        std::cout<< p.n << std::endl;
    }

    for(int i = 0; i < conesides; i++) {
        Mtools::ContactPoint S;
        S.n[0] = (p.n[0]);
        S.n[1] = (p.n[1]+(frictioncoef*cos((2*i*M_PI)/conesides)));
        S.n[2] = (p.n[2]+(frictioncoef*sin((2*i*M_PI)/conesides)));
        S.p = p.p;
        S.id = id;
        S.n.normalize();

        if(printInfo) {
            std::cout << "Loop " << i <<std::endl;
            std::cout << "nCone: " << std::endl;
            std::cout << S.n << std::endl;
            std::cout << "Force: " << std::endl;
            std::cout << S.n.norm() <<" " << id << std::endl;
            std::cout<<std::endl;
        }
        StoreCones.push_back(S);
    }
}




