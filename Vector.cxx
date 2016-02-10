#include "Vector.h"

#include <cmath>
#include <stdio.h>

Vector3D Vector3D::cross(Vector3D otherVec) {
    Vector3D rtn;
    rtn.x = y*otherVec.z - z*otherVec.y;
    rtn.y = z*otherVec.x - x*otherVec.z;
    rtn.z = x*otherVec.y - y*otherVec.x;
    return rtn;
}

Vector3D Vector3D::subtract(Vector3D otherVec) {
    Vector3D rtn;
    rtn.x = x-otherVec.x;
    rtn.y = y-otherVec.y;
    rtn.z = z-otherVec.z;
    return rtn;
}

Vector3D Vector3D::normalize() {
    Vector3D rtn;
    double mag = GetMagnitude();
    rtn.x = x/mag;
    rtn.y = y/mag;
    rtn.z = z/mag;
    return rtn;
}

double Vector3D::GetMagnitude() {
    return sqrt(x*x+y*y+z*z);
}

Vector3D Vector3D::scale(double scale) {
    Vector3D rtn;
    rtn.x = x*scale;
    rtn.y = y*scale;
    rtn.z = z*scale;
    return rtn;
}

Vector3D Vector3D::add(Vector3D otherVec) {
    Vector3D rtn;
    rtn.x = x+otherVec.x;
    rtn.y = y+otherVec.y;
    rtn.z = z+otherVec.z;
    return rtn;
}

void Vector3D::Print() {
    fprintf(stderr, "(%f, %f, %f) -> %f\n", x, y, z, GetMagnitude());
}

Vector3D Vector3D::addScalar(int scalar) {
    return addScalar((double)scalar);
}
Vector3D Vector3D::addScalar(double scalar) {
    Vector3D rtn;
    rtn.x = x+scalar;
    rtn.y = y+scalar;
    rtn.z = z+scalar;
    return rtn;
}
