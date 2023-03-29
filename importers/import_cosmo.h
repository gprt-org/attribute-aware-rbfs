#include <gprt.h>


#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <glm/ext.hpp>
#include <glm/glm.hpp>
// #include "bat_file.h"
// #include "borrowed_array.h"
#include "file_mapping.h"
// #include "lba_tree_builder.h"
// #include "util.h"

#include <algorithm>
#include <filesystem>
namespace fs = std::filesystem;

// #pragma pack(push, 1)
// struct CosmicWebHeader {
//     // The number of particles in this dat file
//     int np_local;
//     float a, t, tau;
//     int nts;
//     float dt_f_acc, dt_pp_acc, dt_c_acc;
//     int cur_checkpoint, cur_projection, cur_halofind;
//     float massp;
// };
// #pragma pack(pop)

// std::ostream &operator<<(std::ostream &os, const CosmicWebHeader &h)
// {
//     os << "{\n"
//        << "  np_local = " << h.np_local << "\n"
//        << "  a = " << h.a << "\n"
//        << "  t = " << h.t << "\n"
//        << "  tau = " << h.tau << "\n"
//        << "  nts = " << h.nts << "\n"
//        << "  dt_f_acc = " << h.dt_f_acc << "\n"
//        << "  dt_pp_acc = " << h.dt_pp_acc << "\n"
//        << "  dt_c_acc = " << h.dt_c_acc << "\n"
//        << "  cur_checkpoint = " << h.cur_checkpoint << "\n"
//        << "  cur_halofind = " << h.cur_halofind << "\n"
//        << "  massp = " << h.massp << "\n}";
//     return os;
// }

#include <sys/stat.h>

long GetFileSize(std::string filename)
{
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

void importCosmo(std::string path, std::vector<std::vector<std::pair<uint64_t, float4>>> &particleData)
{
    std::vector<std::string> cosmos;
    for(auto& p: fs::directory_iterator(path)) {
        cosmos.push_back(p.path());
    }

    // load in alphabetical order
    std::sort(cosmos.begin(), cosmos.end());

    particleData.resize(cosmos.size());


    for (uint32_t fid = 0; fid < cosmos.size(); ++fid) {
        std::cout<<cosmos[fid]<<std::endl;

        int timestep = fid;
        // 0 --> z = 200
        // 624 --> z = 0
        float tf = timestep / 624.0f;
        float zshift = 200.f * tf;//  lerp(200.0f, 0.0f, tf);


        std::ifstream is;
        is.open(cosmos[fid], std::ios::binary);

        is.seekg (0, std::ios::end);
        size_t len = is.tellg();
        is.seekg (0, std::ios::beg);

        size_t numParticles = len / 58;

        particleData[fid].resize(numParticles);

        for (size_t i = 0; i < numParticles; ++i)
        {
            // position and velocity vectors
            /*float x;
            float vx;
            float y;
            float vy;
            float z;
            float vz;
​
​
            // other fields
            float mass;
            float uu;
            float hh;
            float mu;
            float rho;
            float phi;
​
​
            // particle ID and mask
            int64_t id;
            uint16_t mask;*/

            char bytes[58] = { 0 };
            is.read(bytes, sizeof(bytes));

            //if (false) // big endian to little endian
            //{
            //    for (int off = 0; off < 32; off += 4)
            //    {
            //        std::swap(bytes[off + 0], bytes[off + 3]);
            //        std::swap(bytes[off + 1], bytes[off + 2]);
            //    }
            //}

            float x = *reinterpret_cast<float*>(&bytes[ 0]);
            float vx = *reinterpret_cast<float*>(&bytes[ 4]);
            float y=  *reinterpret_cast<float*>(&bytes[ 8]);
            float vy = *reinterpret_cast<float*>(&bytes[12]);
            float z=  *reinterpret_cast<float*>(&bytes[16]);
            float vz = *reinterpret_cast<float*>(&bytes[20]);
            float mass  = *reinterpret_cast<float*>(&bytes[24]);
            float uu  = *reinterpret_cast<float*>(&bytes[28]);
            float hh  = *reinterpret_cast<float*>(&bytes[32]);
            float mu  = *reinterpret_cast<float*>(&bytes[36]);
            float rho  = *reinterpret_cast<float*>(&bytes[40]);
            float phi  = *reinterpret_cast<float*>(&bytes[44]);
            int64_t id  = *reinterpret_cast<float*>(&bytes[48]);
            float mask  = *reinterpret_cast<float*>(&bytes[46]);

            float T = 4.8e5 * uu / ((1+zshift)*(1+zshift)*(1+zshift));

            float4 point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.w = length(float3(vx, vy, vz));

            particleData[fid][i].second = point;

            //std::cout << T << '\n';

            //std::cout << x << ' ' << y << ' ' << z << '\n';
            // if (1)//T > FLT_EPSILON)
            // {
            //     list[i] = make_sphere(vec3(x, y, z), 0.05);
            //     materials[i] = make_blackbody(max(0.0f, T/25.f));

            //     auto mat = *materials[i].as<emissive<float>>();
            //     area_light<float, basic_sphere<float>> light(list[i]);
            //     light.set_cl(to_rgb(mat.ce()));
            //     light.set_kl(mat.ls());
            //     lights.push_back(light);
            // }
            // else
            // {
            //     list[i] = make_sphere(vec3(x, y, z), 0.1);
            //     //materials[i] = make_metal(vec3(0.8f));
            //     materials[i] = make_lambertian(vec3(0.0001f));
            // }

        }
    }
}