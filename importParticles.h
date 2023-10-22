#include <iostream>
#include <fstream>
#include <vector>
#include <gprt.h>

// This function returns by reference a list of timesteps, where each timestep is a list of particles.
// A particle is defined by a float4, where the "w" component acts as the scalar "attribute"
std::vector<std::vector<float4>> importParticles(std::string path)
{
    std::vector<std::vector<float4>> particles;
    // This importer first looks at a header file which lists a series of timesteps
    std::ifstream hrf(path);
    std::string timestepName;
    while (std::getline (hrf, timestepName)) {
        // We assume the path to a binary particle format can be found by concatinating an
        // underscore, then the actual timestep name (usually a number).
        std::ifstream rf(path + "_" + timestepName, std::ios::out | std::ios::binary);    

        // From here, we assume a binary format.

        // The first 64 bytes encode the number of particles in the file.
        uint64_t size;
        rf.read((char *) &size, sizeof(uint64_t));

        // The remaining binary data are a series of float four components.
        // The first three floats are XYZ coordinates, and the fourth is a scalar attribute.
        particles.push_back(std::vector<float4>());     
        particles[particles.size() - 1].resize(size);
        rf.read((char *) particles[particles.size() - 1].data(), size * sizeof(float4));
    }
    return particles;
}