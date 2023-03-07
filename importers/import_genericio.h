#ifdef GENERICIO

#endif

#include <gprt.h>
void importCosmo(std::string path, std::vector<std::pair<uint64_t, float4>> &particleData)
{
    #ifdef GENERICIO

    std::cerr << "Not yet supported" << std::endl;
    std::cerr << program;
    std::exit(1);

    #else
    std::cerr << "GenericIO support not compiled into viewer" << std::endl;
    std::cerr << program;
    std::exit(1);
    #endif
}