#include "Mtools.h"

Mtools::Quaternion Mtools::getRot(const Eigen::Vector3f &from, const Eigen::Vector3f &to){

    Eigen::Vector3f fN = from;
    fN.normalize();
    Eigen::Vector3f tN = to;
    tN.normalize();

    float d = fN.dot(tN);
    Eigen::Vector3f crossvec = fN.cross(tN);
    float crosslen = crossvec.norm();
    Mtools::Quaternion q;

    if(crosslen == 0.0f) {
        //Parallel vectors
        if(d < 0.0f){
            //Parallel and pointing in oposite direction
            //Crossing with X axis

            Eigen::Vector3f t = fN.cross(Eigen::Vector3f(1.0f,0.0f,0.0f));
            //no->cross with y axis
            if(t.norm() == 0.0f) {
                t = fN.cross(Eigen::Vector3f(0.0f,1.0f,0.0f));
            }
            t.normalize();
            q.x = t[0];
            q.y = t[1];
            q.z = t[2];
            q.w = 0.0f;
        }
    }
    else {
        //not parallel
        crossvec.normalize();
        crossvec *= (float)sqrt(0.5f * fabs(1.0f - d));
        q.x = crossvec[0];
        q.y = crossvec[1];
        q.z = crossvec[2];
        q.w = (float)sqrt(0.5f * fabs(1.0 + d));
    }
    return q;
}

Eigen::Matrix4f Mtools::quat2eigen4f(const Quaternion q) {
    return quat2eigen4f(q.x, q.y, q.z, q.w);
}

Eigen::Matrix4f Mtools::quat2eigen4f(float x, float y, float z, float w) {
    Eigen::Matrix4f m;
    m.setIdentity();
    Eigen::Quaternionf q(w, x, y, z);
    Eigen::Matrix3f m3;
    m3 = q.toRotationMatrix();
    m.block(0,0,3,3) = m3;
    return m;
}

Mtools::Quaternion Mtools::eigen4f2quat(const Eigen::Matrix4f &m) {

    Eigen::Matrix3f m3 = m.block(0,0,3,3);
    Eigen::Quaternionf q(m3);
    Quaternion qr;
    qr.x = q.x();
    qr.y = q.y();
    qr.z = q.z();
    qr.w = q.w();

    return qr;
}

float Mtools::getAng(const Quaternion &q) {
    float n = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if(n < 1e-10) 
        return 0.0f;
    n = 1.0f / n;
    return (float)(2.0f * acosf(q.w * n));
}

Eigen::Vector3f Mtools::TPosition(const Eigen::Vector3f &pos, const Eigen::Matrix4f &m) {
    Eigen::Vector4f t(pos.x(), pos.y(), pos.z(), 1);
    t = m * t;
    return t.head(3);
}

float Mtools::calculateQuality(float GWSOffset, float OWSOffset){
    float Quality;
    if(OWSOffset != 0) Quality = GWSOffset / OWSOffset;
    else Quality = GWSOffset;
    return Quality;
}



