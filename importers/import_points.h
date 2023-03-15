#include <iostream>
#include <fstream>
#include <vector>
#include <gprt.h>
void importPoints(std::string path, std::vector<std::vector<std::pair<uint64_t, float4>>> &particleData)
{
    std::ifstream hrf(path);
    std::string timestepName;
    while (std::getline (hrf, timestepName)) {
        // Output the text from the file
        
        std::ifstream rf(path + "_" + timestepName, std::ios::out | std::ios::binary);    
        uint64_t size;
        rf.read((char *) &size, sizeof(uint64_t));
        std::vector<float4> points(size);
        rf.read((char *) points.data(), points.size() * sizeof(float4));

        particleData.push_back(std::vector<std::pair<uint64_t, float4>>());
     
        particleData[particleData.size() - 1].resize(points.size());
        for (uint64_t i = 0; i < points.size(); ++i) {
            particleData[particleData.size() - 1][i].second = points[i];
        }

    }
}