#include <iostream>

#include <vtkPNGWriter.h>
#include <vtkGenericDataObjectReader.h>

#include "Raycaster.h"
#include "Camera.h"

using std::cerr;
using std::endl;

#define OUT cerr

Camera
SetupCamera(void)
{
    Camera rv;
    rv.focus.x = 0;
    rv.focus.y = 0;
    rv.focus.z = 0;
    rv.up.x = 0;
    rv.up.y = 1;
    rv.up.z = 0;
    rv.angle = 30;
    rv.near = 7.5e+7;
    rv.far = 1.4e+8;
    rv.position.x = -8.25e+7;
    rv.position.y = -3.45e+7;
    rv.position.z = 3.35e+7;

    return rv;
}

int main(int argc, char **argv) {

    if(argc < 2) {
        OUT << "Usage: " << argv[0] << " <File to render>" << endl;
        exit(EXIT_FAILURE);
    }

    vtkGenericDataObjectReader *reader = vtkGenericDataObjectReader::New();
    reader->SetFileName(argv[1]);

    OUT << "Reading data file: " << argv[1] << endl;

    reader->Update();
    if(!reader->IsFileRectilinearGrid()) {
        OUT << "Input file must be a rectilinear grid!" << endl;
        exit(EXIT_FAILURE);
    }

    Raycaster caster;
    caster.SetCamera(SetupCamera());
    caster.SetOutputDimensions(1024, 1024);
    caster.SetInputData(reader->GetRectilinearGridOutput());
    caster.SetSampleRate(500);

    caster.Update();

    vtkPNGWriter *writer = vtkPNGWriter::New();
    writer->SetFileName("Out.png");
    writer->SetInputConnection(caster.GetOutput());
    OUT << "Writing to Out.png" << endl;
    writer->Write();

}
