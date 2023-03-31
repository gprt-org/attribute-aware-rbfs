#include <algorithm>
#include <filesystem>
#ifdef VTK
#include <vtkSmartPointer.h>
#include <vtkPointData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridReader.h>
#endif
#include "import_vtk.h"
namespace fs = std::filesystem;
void importVTK(std::string vtkPath, std::vector<std::vector<std::pair<uint64_t, float4>>> &particleData)
{
  #ifdef VTK
  std::vector<std::string> vtus;
  for(auto& p: fs::directory_iterator(vtkPath)) {
    if (p.path().extension() == ".vtu") {
      vtus.push_back(p.path());
    }
  }

  // load in alphabetical order
  std::sort(vtus.begin(), vtus.end());

  for (auto &vtu : vtus) {
    const char *fileName = vtu.c_str();
    auto reader = vtkSmartPointer<vtkXMLUnstructuredGridReader>::New();
    reader->SetFileName(fileName);
    reader->Update();

    auto ug = reader->GetOutput();
    auto data = ug->GetPointData()->GetArray("concentration");
    if (!data) continue;

    std::cout << "Loading " << fileName << '\n';
    //ug->PrintSelf(cout, vtkIndent(0));
    //std::cout << "\n";
    //data->PrintSelf(cout, vtkIndent(0));
    //std::cout << "\n";

    int numPoints = ug->GetNumberOfPoints();
    std::cout<<"Num points "<< numPoints << std::endl;
    if (numPoints > 0) {
      particleData.emplace_back(std::vector<std::pair<uint64_t, float4>>(numPoints));
      auto &particles = particleData.back();

      for (int i=0; i<numPoints; ++i) {
        double X[3];
        ug->GetPoint(i, X);
        float4 P;
        P.x = X[0]; P.y = X[1]; P.z = X[2];
        P.w = data->GetComponent(i, 0);
        //std::cout << P << '\n';
        particles[i] = {i,P};
      }
    }
  }
  #else
  std::cerr << "VTK support not compiled into viewer" << std::endl;
  std::exit(1);
  #endif
}
