#ifndef RAYCASTER_H
#define RAYCASTER_H

#include <vtkRectilinearGrid.h>
#include <vtkImageData.h>
#include <vtkAlgorithmOutput.h>
#include <vtkImageImport.h>

#include "Camera.h"
#include "Vector.h"

class TransferFunction
{
  public:
    double          min;
    double          max;
    int             numBins;
    unsigned char  *colors;  // size is 3*numBins
    double         *opacities; // size is numBins

    // Take in a value and applies the transfer function.
    // Step #1: figure out which bin "value" lies in.
    // If "min" is 2 and "max" is 4, and there are 10 bins, then
    //   bin 0 = 2->2.2
    //   bin 1 = 2.2->2.4
    //   bin 2 = 2.4->2.6
    //   bin 3 = 2.6->2.8
    //   bin 4 = 2.8->3.0
    //   bin 5 = 3.0->3.2
    //   bin 6 = 3.2->3.4
    //   bin 7 = 3.4->3.6
    //   bin 8 = 3.6->3.8
    //   bin 9 = 3.8->4.0
    // and, for example, a "value" of 3.15 would return the color in bin 5
    // and the opacity at "opacities[5]".
    void Apply(double value, unsigned char *RGB, double &opacity)
    {
        if(value < min || value > max) {
            RGB[0] = 0;
            RGB[1] = 0;
            RGB[2] = 0;
            opacity = 0;
            return;
        }
        double dBin = (max-min)/numBins;
        int bin = (value-min)/dBin;
        //fprintf(stderr, "range: %f->%f, val=%f\n", min, max, value);
        //fprintf(stderr, "bin %d/%d\n", bin, numBins);
        RGB[0] = colors[3*bin+0];
        RGB[1] = colors[3*bin+1];
        RGB[2] = colors[3*bin+2];
        opacity = opacities[bin];
    }
};

class Raycaster {
  private:
    vtkRectilinearGrid *data;
    vtkImageImport *importer;
    TransferFunction transferFunc;
    int sampleRate;
    Camera cam;
    int width, height;
    unsigned char *output;

    void SetupTransferFunction();
    Vector3D GetRayForPixel(int, int);
  public:
         Raycaster();
        ~Raycaster();

    void SetOutputDimensions(int, int);
    void SetInputData(vtkRectilinearGrid *);
    void Update();
    void SetCamera(Camera);
    void SetSampleRate(int);
    void CastRay(int x, int y);
    int GetWidth();

    vtkAlgorithmOutput *GetOutput();
};

#endif
