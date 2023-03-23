#ifdef GENERICIO

#endif

#include <gprt.h>
#include <GenericIO.h>
void importCosmo(std::string path, std::vector<std::vector<std::pair<uint64_t, float4>>> &particleData)
{
    #ifdef GENERICIO
    gio::GenericIO IO(path, gio::GenericIO::FileIOPOSIX);
    IO.openAndReadHeader(gio::GenericIO::MismatchRedistribute);

    size_t np = IO.readNumElems();
    size_t readsize_float = np + IO.requestedExtraSpace()/4;

    std::vector<float> x(readsize_float);
    std::vector<float> y(readsize_float);
    std::vector<float> z(readsize_float);

    IO.addVariable("x", x, gio::GenericIO::VarHasExtraSpace);
    IO.addVariable("y", y, gio::GenericIO::VarHasExtraSpace);
    IO.addVariable("z", z, gio::GenericIO::VarHasExtraSpace);

    IO.readData();

    x.resize(np);
    y.resize(np);
    z.resize(np);
    #else
    std::cerr << "GenericIO support not compiled into viewer" << std::endl;
    //std::cerr << program;
    std::exit(1);
    #endif
}
