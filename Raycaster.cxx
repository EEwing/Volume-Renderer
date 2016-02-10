#include "Raycaster.h"

#include <pthread.h>

#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkImageImport.h>

#define PI 3.14159256358979

#define NUM_THREADS 8

Raycaster::Raycaster() {
    this->output = NULL;
    this->data = NULL;

    this->importer = vtkImageImport::New();
    this->importer->SetDataSpacing(1, 1, 1);
    this->importer->SetDataOrigin(0, 0, 0);
    this->importer->SetDataScalarTypeToUnsignedChar();


    SetupTransferFunction();
/*
    this->output = vtkImageData::New();
    this->output->SetOrigin(0, 0, 0);
    this->output->SetSpacing(1.0, 1.0, 1.0);
    this->output->SetScalarTypeToUnsignedChar();
    this->output->SetNumberOfScalarComponents(3);
*/
}

Raycaster::~Raycaster() {
    if(this->output)
        delete [] output;
    //this->output->Delete();
}

void Raycaster::SetOutputDimensions(int w, int h) {
    if(this->output)
        delete [] output;
    this->output = new unsigned char[w*h*3];
    this->importer->SetWholeExtent(0, w-1, 0, h-1, 0, 0);
    this->importer->SetDataExtentToWholeExtent();
    this->importer->SetNumberOfScalarComponents(3);
    this->importer->SetImportVoidPointer(this->output);
    this->width = w;
    this->height = h;
}

void Raycaster::SetInputData(vtkRectilinearGrid *data) {
    this->data = data;
}

Vector3D Raycaster::GetRayForPixel(int x, int y) {
    Vector3D look = cam.focus.subtract(cam.position);
    Vector3D ru = look.cross(cam.up).normalize();
    Vector3D rv = ru.cross(look).normalize();
    Vector3D rdx = ru.scale((2*tan(cam.angle*PI/360.))/width);
    Vector3D rdy = rv.scale((2*tan(cam.angle*PI/360.))/height);
    Vector3D ray = look.normalize().add(rdx.scale((2*x+1-width)/2)).add(rdy.scale((2*y+1-height)/2));
    return ray;
}

double GetValueAtPoint(vtkRectilinearGrid *data, Vector3D pos) {
    int *dims = data->GetDimensions();
    float *X = (float *) data->GetXCoordinates()->GetVoidPointer(0);
    float *Y = (float *) data->GetYCoordinates()->GetVoidPointer(0);
    float *Z = (float *) data->GetZCoordinates()->GetVoidPointer(0);
    float *F = (float *) data->GetPointData()->GetScalars()->GetVoidPointer(0);

    off_t cellidx = 0;
    off_t cellidy = 0;
    off_t cellidz = 0;

    while(X[cellidx+1] <= pos.x && cellidx < dims[0]-1) {
        ++cellidx;
    }

    while(Y[cellidy+1] <= pos.y && cellidy < dims[1]-1) {
        ++cellidy;
    }

    while(Z[cellidz+1] <= pos.z && cellidz < dims[2]-1) {
        ++cellidz;
    }

    float tx = ( pos.x - X[cellidx] )/( X[cellidx+1] - X[cellidx] );
    float ty = ( pos.y - Y[cellidy] )/( Y[cellidy+1] - Y[cellidy] );
    float tz = ( pos.z - Z[cellidz] )/( Z[cellidz+1] - Z[cellidz] );

    // Check for out of bounds elements
    if( tx < 0 || tx > 1 || ty < 0 || ty > 1 || tz < 0 || tz > 1) {
        return 0;
    }

    int ptidx[3] = {cellidx, cellidy, cellidz};

    //return idx[1]*dims[0]+idx[0];
    int pt000 = cellidz*dims[0]*dims[1]+cellidy*dims[0]+cellidx;
    int pt001 = pt000+1;
    int pt010 = pt000+dims[0];
    int pt011 = pt010+1;

    int pt100 = pt000+dims[0]*dims[1];
    int pt101 = pt100+1;
    int pt110 = pt100+dims[0];
    int pt111 = pt110+1;

    float v000 = F[pt000];
    float v001 = F[pt001];
    float v010 = F[pt010];
    float v011 = F[pt011];

    float v100 = F[pt000];
    float v101 = F[pt001];
    float v110 = F[pt010];
    float v111 = F[pt011];

    // Interpolate z=0 plane
    float val1 = v000 + tx*(v001-v000);
    float val2 = v010 + tx*(v011-v010);
    float val3 = val1 + ty*(val2-val1);
    
    // Interpolate z=1 plane
    float val4 = v100 + tx*(v101-v100);
    float val5 = v110 + tx*(v111-v110);
    float val6 = val4 + ty*(val5-val4);

    // Interpolate along z
    float rtn = val3 + tz*(val6-val3);


    //fprintf(stderr, "Point ");
    //pos.Print();
    //fprintf(stderr, "\tlies in cell (%ld, %ld, %ld)\n", cellidx, cellidy, cellidz);
    //fprintf(stderr, "\t\tData ranges %f->%f %f->%f %f->%f\n", X[cellidx], X[cellidx+1], Y[cellidy], Y[cellidy+1], Z[cellidz], Z[cellidz+1]);
    //fprintf(stderr, "\t\tproportions %f %f %f\n", tx, ty, tz);
    return rtn; // IMPLEMENT ME!!
}

struct Thread_Info {
    Raycaster *caster;
    int start, length;
    //int x, y;
};
void *CastRayOnThread(void *args) {
    Thread_Info *ti = (Thread_Info *)args;
    Raycaster *caster = ti->caster;
    for(int i=ti->start; i<ti->start+ti->length; ++i)
        caster->CastRay(i%caster->GetWidth(), i/caster->GetWidth());
}

void LayerColors(unsigned char *f, double &fA, unsigned char *b, double bA) {
    double newOpacity = fA+(1-fA)*bA;
    for(int i=0; i<3; ++i) {
        //f[i] = f[i] + (1-fA)*b[i]*bA;
        f[i] = (fA*f[i]+(1-fA)*b[i]*bA)/newOpacity;
    }
    fA = newOpacity;
    //ptr[0] = ptr[1] = 0;
}

void Raycaster::CastRay(int x, int y) {
    double dSample = (cam.far - cam.near)/sampleRate;

    //fprintf(stderr, "Calculating pixel (%d, %d)\n", x, y);

    unsigned char *ptr = &output[(y*width+x)*3];
    ptr[0] = 255;
    ptr[1] = 255;
    ptr[2] = 255;
    Vector3D ray = GetRayForPixel(x, y).normalize();
    Vector3D pointOnPlane = cam.position.add(ray.scale(cam.near));

    double fA = 0;
    if(x < transferFunc.numBins && y < 10) {
        ptr[0] = transferFunc.colors[3*x];
        ptr[1] = transferFunc.colors[3*x+1];
        ptr[2] = transferFunc.colors[3*x+2];
        return;
    }
    if(x < transferFunc.numBins && y<20) {
        ptr[0] = transferFunc.opacities[x]*255;
        ptr[1] = transferFunc.opacities[x]*255;
        ptr[2] = transferFunc.opacities[x]*255;
        return;
    }

    double *bbox = data->GetBounds();
/*
    double tx[2], ty[2], tz[2];
    tx[0] = (bbox[0]-pointOnPlane.x)/ray.x;
    tx[1] = (bbox[1]-pointOnPlane.x)/ray.x;
    ty[0] = (bbox[2]-pointOnPlane.y)/ray.y;
    ty[1] = (bbox[3]-pointOnPlane.y)/ray.y;
    tz[0] = (bbox[4]-pointOnPlane.z)/ray.z;
    tz[1] = (bbox[5]-pointOnPlane.z)/ray.z;

    if(tx[0] > ty[1] || ty[0] > tx[1]) { 
        fprintf(stderr, "Doesn't pass 2d test\n");
        return;
    }

    if(ty[0] > tx[0])
        tx[0] = ty[0];

    if(ty[1] < tx[1])
        tx[1] = ty[1];

    if(tx[0] > tz[1] || tz[0] > tx[1])
        return;
*/

    float tmin = -1e15, tmax = 1e15;
    float t1, t2;


    if (ray.x != 0.0) {
        t1 = (bbox[0] - pointOnPlane.x) / ray.x;
        t2 = (bbox[1] - pointOnPlane.x) / ray.x;
 
        tmin = fmax ( tmin, fmin (t1, t2) );
        tmax = fmin ( tmax, fmax (t1, t2) );
    } else {
        if ((pointOnPlane.x < bbox[0]) || (pointOnPlane.x > bbox[1]))
            return;
    }
    if (ray.y != 0.0) {
        t1 = (bbox[2] - pointOnPlane.y) / ray.y;
        t2 = (bbox[3] - pointOnPlane.y) / ray.y;
 
        tmin = fmax ( tmin, fmin (t1, t2) );
        tmax = fmin ( tmax, fmax (t1, t2) );
    } else {
        if ((pointOnPlane.y < bbox[2]) || (pointOnPlane.y > bbox[3]))
            return;
    }
    if (ray.z != 0.0) {
        t1 = (bbox[4]   - pointOnPlane.z) / ray.z;
        t2 = (bbox[5] - pointOnPlane.z) / ray.z;
 
        tmin = fmax ( tmin, fmin (t1, t2) );
        tmax = fmin ( tmax, fmax (t1, t2) );
    } else {
        if ((pointOnPlane.z < bbox[4]) || (pointOnPlane.z > bbox[5]))
            return;
    }

    if (tmin >= tmax)
        return;
    else if (tmin > 0) {
        //return tmin;
    } else if (tmax > 0) {
        //return tmax;
    } else
        return;

    unsigned char f[3];
    for(int curSample=0; curSample<sampleRate; ++curSample) {
        Vector3D samplePos = pointOnPlane.add(ray.scale(dSample*curSample));
        double value = GetValueAtPoint(data, samplePos);
        unsigned char b[3];
        double opacity;
        transferFunc.Apply(value, b, opacity);
        if(opacity == 0)
            continue;
        double bA = 1 - pow(1 - opacity, 500./sampleRate);
        //double bA = opacity;
        LayerColors(f, fA, b, bA);
        //fprintf(stderr, "Accumulating data as: (%d, %d, %d, %f)\n", ptr[0], ptr[1], ptr[2], fA);
    }
    unsigned char white[3] = {255, 255, 255};
    LayerColors(f, fA, white, 1);
    ptr[0] = fA*f[0];
    ptr[1] = fA*f[1];
    ptr[2] = fA*f[2];
}

void Raycaster::Update() { 
    pthread_t threads[NUM_THREADS];
    Thread_Info ti[NUM_THREADS];

    int pixPerThread = width*height/NUM_THREADS;
    int numLargeThreads = width*height-pixPerThread*NUM_THREADS;

    int start = 0;
    for(int i=0; i<NUM_THREADS; ++i) {
        ti[i].start = start;
        if(i < numLargeThreads)
            ti[i].length = pixPerThread+1;
        else
            ti[i].length = pixPerThread;
        start += ti[i].length;
        ti[i].caster = this;
        pthread_create(&threads[i], NULL, CastRayOnThread, &ti[i]);
    }
    for(int i=0; i<NUM_THREADS; ++i)
        pthread_join(threads[i], NULL);
    
    this->importer->Update();
}

vtkAlgorithmOutput *Raycaster::GetOutput() {
    return this->importer->GetOutputPort();
}

void Raycaster::SetupTransferFunction(void)
{
    int  i;

    transferFunc.min = 10;
    transferFunc.max = 15;
    transferFunc.numBins = 256;
    transferFunc.colors = new unsigned char[3*256];
    transferFunc.opacities = new double[256];
    unsigned char charOpacity[256] = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 4, 3, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 17, 17, 17, 17, 17, 17, 16, 16, 15, 14, 13, 12, 11, 9, 8, 7, 6, 5, 5, 4, 3, 3, 3, 4, 5, 6, 7, 8, 9, 11, 12, 14, 16, 18, 20, 22, 24, 27, 29, 32, 35, 38, 41, 44, 47, 50, 52, 55, 58, 60, 62, 64, 66, 67, 68, 69, 70, 70, 70, 69, 68, 67, 66, 64, 62, 60, 58, 55, 52, 50, 47, 44, 41, 38, 35, 32, 29, 27, 24, 22, 20, 20, 23, 28, 33, 38, 45, 51, 59, 67, 76, 85, 95, 105, 116, 127, 138, 149, 160, 170, 180, 189, 198, 205, 212, 217, 221, 223, 224, 224, 222, 219, 214, 208, 201, 193, 184, 174, 164, 153, 142, 131, 120, 109, 99, 89, 79, 70, 62, 54, 47, 40, 35, 30, 25, 21, 17, 14, 12, 10, 8, 6, 5, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        };

    for (i = 0 ; i < 256 ; i++)
        transferFunc.opacities[i] = charOpacity[i]/255.0;
    const int numControlPoints = 8;
    unsigned char controlPointColors[numControlPoints*3] = { 
           71, 71, 219, 0, 0, 91, 0, 255, 255, 0, 127, 0, 
           255, 255, 0, 255, 96, 0, 107, 0, 0, 224, 76, 76 
       };
    double controlPointPositions[numControlPoints] = { 0, 0.143, 0.285, 0.429, 0.571, 0.714, 0.857, 1.0 };
    for (i = 0 ; i < numControlPoints-1 ; i++)
    {
        int start = controlPointPositions[i]*transferFunc.numBins;
        int end   = controlPointPositions[i+1]*transferFunc.numBins+1;
cerr << "Working on " << i << "/" << i+1 << ", with range " << start << "/" << end << endl;
        if (end >= transferFunc.numBins)
            end = transferFunc.numBins-1;
        for (int j = start ; j <= end ; j++)
        {
            double proportion = (j/(transferFunc.numBins-1.0)-controlPointPositions[i])/(controlPointPositions[i+1]-controlPointPositions[i]);
            if (proportion < 0 || proportion > 1.)
                continue;
            for (int k = 0 ; k < 3 ; k++)
                transferFunc.colors[3*j+k] = proportion*(controlPointColors[3*(i+1)+k]-controlPointColors[3*i+k])
                                 + controlPointColors[3*i+k];
        }
    }    
}

void Raycaster::SetCamera(Camera cam) {
    this->cam = cam;
}

void Raycaster::SetSampleRate(int rate) {
    this->sampleRate = rate;
}

int Raycaster::GetWidth() { return width; }
