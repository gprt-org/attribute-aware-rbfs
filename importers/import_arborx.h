#ifdef ARBORX
// For importing HACC data
#include <ArborX_DBSCAN.hpp>
using ArborX::ExperimentalHyperGeometry::Point;
template <int DIM>
std::vector<Point<DIM>> loadData(std::string const &filename,
                                 bool binary = true, int max_num_points = -1)
{
  std::cout << "Reading in \"" << filename << "\" in "
            << (binary ? "binary" : "text") << " mode...";
  std::cout.flush();

  std::ifstream input;
  if (!binary)
    input.open(filename);
  else
    input.open(filename, std::ifstream::binary);
  ARBORX_ASSERT(input.good());

  std::vector<Point<DIM>> v;

  int num_points = 0;
  int dim = 0;
  if (!binary)
  {
    input >> num_points;
    input >> dim;
  }
  else
  {
    input.read(reinterpret_cast<char *>(&num_points), sizeof(int));
    input.read(reinterpret_cast<char *>(&dim), sizeof(int));
  }

  ARBORX_ASSERT(dim == DIM);

  if (max_num_points > 0 && max_num_points < num_points)
    num_points = max_num_points;

  v.resize(num_points);
  if (!binary)
  {
    auto it = std::istream_iterator<float>(input);
    for (int i = 0; i < num_points; ++i)
      for (int d = 0; d < DIM; ++d)
        v[i][d] = *it++;
  }
  else
  {
    // Directly read into a point
    input.read(reinterpret_cast<char *>(v.data()),
               num_points * sizeof(Point<DIM>));
  }
  input.close();
  std::cout << "done\nRead in " << num_points << " " << dim << "D points"
            << std::endl;

  return v;
}
#endif

void importArborX(std::string path, std::vector<std::vector<std::pair<uint64_t, float4>>> &particleData)
{
    #ifdef ARBORX
    std::cout << path << std::endl;
    auto result = loadData<3>(path);

    // just one frame for now
    particleData.resize(1);

    // Add particles to our particle array, computing AABB along the way 
    particleData[0].resize(result.size());

    for (size_t i = 0; i < result.size(); ++i) {    
      particleData[0][i].second = float4(result[i][0], result[i][1], result[i][2], float(i) / float(result.size()));
    }
    #else
    std::cerr << "ARBORX support not compiled into viewer" << std::endl;
    //std::cerr << program;
    std::exit(1);
    #endif
}
