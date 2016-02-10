#ifndef CAMERA_H
#define CAMERA_H

#include "Vector.h"

struct Camera
{
    double          near, far;
    double          angle;
    Vector3D        position;
    Vector3D        focus;
    Vector3D        up;
    //double          position[3];
    //double          focus[3];
    //double          up[3];
};

#endif

/*
int main()
{
    TransferFunction tf = SetupTransferFunction();
    for (int i = 0 ; i < tf.numBins ; i++)
    {
        cerr << i << ": " << (int) tf.colors[3*i] << ", " << (int) tf.colors[3*i+1] << ", " << (int) tf.colors[3*i+2] << ", " << tf.opacities[i] << endl;
    }
}
 */
