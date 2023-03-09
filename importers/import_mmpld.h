#ifdef MMPLD
#include "mmpld.h"
float4 getMMPLDParticle(const char* baseptr, mmpld::vertex_type const& vt, mmpld::color_type const& ct)
{
    float4 ret;

    if (vt == mmpld::vertex_type::FLOAT_XYZ || vt == mmpld::vertex_type::FLOAT_XYZR) {
        auto const fptr = reinterpret_cast<float const*>(baseptr);
        ret = float4(fptr[0], fptr[1], fptr[2], 0.f);
    } else if (vt == mmpld::vertex_type::DOUBLE_XYZ) {
        auto const dptr = reinterpret_cast<double const*>(baseptr);
        ret = float4(dptr[0], dptr[1], dptr[2], 0.f);
    }

    if (ct == mmpld::color_type::FLOAT_I) {
        auto const fptr = reinterpret_cast<float const*>(baseptr);
        ret.w = fptr[3];
    }

    return ret;
}

#endif

#include <gprt.h>
void importMMPLD(std::string path, std::vector<std::vector<std::pair<uint64_t, float4>>> &particleData)
{
    #ifdef MMPLD

    mmpld::frame_t frame;
    // mmpld::frame_t frame2;

    mmpld::mmpld file(path);
    particleData.resize(file.GetFrameCount());

    for (int frameID = 0; frameID < particleData.size(); ++frameID) {
        frame = file.ReadFrame(frameID);
        auto pl_count = frame.data.size();
        float tmp = 0.f;
        for (size_t plidx = 0; plidx < pl_count; ++plidx) {
            auto const& entry = frame.data[plidx];
            auto const& vt = entry.list_header.vert_type;
            auto const& ct = entry.list_header.col_type;
            if (!mmpld::HasData(vt)) continue;

            auto const pcount = entry.list_header.particle_count;
            auto const stride = entry.vertex_stride + entry.color_stride;

            for (size_t pidx = 0; pidx < pcount; ++pidx) {
                // Particle part;
                float4 particle;

                auto base_ptr = entry.data.data()+pidx*stride;
                particle = getMMPLDParticle(base_ptr, vt, ct);
                
                // if (particle.w == 0.f) particle.w = tmp; // placeholder
                particleData[frameID].push_back({0, particle});
                // model->particles.push_back(part);
                tmp++;
            }
        }
    }

    // normalize attributes

    // for (uint32_t i = 0; i < particleData.size(); ++i) {
    //     particleData[i].second.w /= tmp;
    // }

    #else
    std::cerr << "MMPLD support not compiled into viewer" << std::endl;
    std::cerr << program;
    std::exit(1);
    #endif
}