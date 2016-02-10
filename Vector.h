#ifndef VECTOR3D_H
#define VECTOR3D_H

struct Vector3D {
    double x, y, z;

    Vector3D add(Vector3D);
    Vector3D addScalar(int);
    Vector3D addScalar(double);
    Vector3D subtract(Vector3D);
    Vector3D cross(Vector3D);
    Vector3D normalize();
    double GetMagnitude();
    Vector3D scale(double);
    void Print();
};


#endif
