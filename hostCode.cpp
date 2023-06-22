// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This program sets up a single geometric object, a mesh for a cube, and
// its acceleration structure, then ray traces it.

// public GPRT API
#include <gprt.h>

// stb
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

// our shared data structures between host and device
#include "sharedCode.h"

#ifdef HEADLESS
#include "ColorMap.h"
#include "generateFibonacciSphere.h"
#else
#include "imgui.h"
#include <imgui_gradient/imgui_gradient.hpp>
#endif

#include <fstream>

#include "IniFile.h"
#include "importers/import_points.h"
#include <argparse/argparse.hpp>

// For parallel sorting of points along a hilbert curve
#include "hilbert.h"
#include "timer.h"
#include <algorithm>
#include <execution>

#define LOG(message)                                                           \
  std::cout << GPRT_TERMINAL_BLUE;                                             \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                        \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                       \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram deviceCodeCommon;
extern GPRTProgram deviceCodeSplat;
extern GPRTProgram deviceCodeRBF;
extern GPRTProgram deviceCodeVoxel;

// initial image resolution
// const int2 fbSize = {1334, 574}; // teaser size
const int2 fbSize = {1024, 1024}; // benchmark size
// const int2 fbSize = {1920, 1080};

// Initial camera parameters
float3 lookFrom = {3.5f, 3.5f, 3.5f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

// uint32_t particlesPerLeaf = 16;
uint32_t particlesPerLeaf = 1;
// float rbfRadius = .01f; //3.f;
// float rbfRadius = 3.f; // 3.f;
std::vector<std::vector<float4>> particles;
size_t maxNumParticles;

uint32_t structuredGridResolution = 256;

static std::vector<std::string> string_split(std::string s, char delim) {
  std::vector<std::string> result;
  std::istringstream stream(s);

  for (std::string token; std::getline(stream, token, delim);) {
    result.push_back(token);
  }

  return result;
}

// Prints to the provided buffer a nice number of bytes (KB, MB, GB, etc)
void pretty_bytes(uint32_t bytes) {
  const char *suffixes[7];
  suffixes[0] = "B";
  suffixes[1] = "KB";
  suffixes[2] = "MB";
  suffixes[3] = "GB";
  suffixes[4] = "TB";
  suffixes[5] = "PB";
  suffixes[6] = "EB";
  uint32_t s = 0; // which suffix to use
  double count = bytes;
  while (count >= 1024 && s < 7) {
    s++;
    count /= 1024;
  }
  if (count - floor(count) == 0.0)
    printf("%d %s", (int)count, suffixes[s]);
  else
    printf("%.1f %s", count, suffixes[s]);
}

#include <iostream>
int main(int argc, char *argv[]) {
  argparse::ArgumentParser program("RT Point Clouds");

  program.add_argument("--points")
      .help("A path to our custom points dataset (ending in .points)")
      .default_value("");

  program.add_argument("--camera")
      .nargs(10)
      .help("posx, posy, posz, atx, aty, atz, upx, upy, upz, fovy")
      .default_value(std::vector<float>{})
      .scan<'g', float>();

  program.add_argument("--radius")
      .help("RBF radius")
      .default_value(0.f)
      .scan<'g', float>();

  program.add_argument("--particles-per-leaf")
      .help("Particles per leaf")
      .default_value(0U)
      .scan<'u', uint32_t>();

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }

  IniFile ini("viewer.ini");
  if (ini.good())
    std::cout << "Found viewer.ini\n";

  std::vector<std::vector<std::pair<uint64_t, float4>>> particleData;

  std::string pointsPath = program.get<std::string>("--points");
  bool synthetic = false;
  if (pointsPath != "") {
    std::cout << "loading " << pointsPath << std::endl;
    importPoints(pointsPath, particleData);
  } else {
    synthetic = true;
    particleData.resize(500);
    for (uint32_t frame = 0; frame < particleData.size(); ++frame) {
      float r = 1;
      particleData[frame].resize(10 * 10 * 10);
      for (int z = 0; z < 10; ++z) {
        for (int y = 0; y < 10; ++y) {
          for (int x = 0; x < 10; ++x) {
            uint32_t i = x + y * 10 + z * 10 * 10;
            float t1 = float(y) / float(10.f);
            float t2 = (float(frame) / float(particleData.size()));

            particleData[frame][i].second =
                float4((sin(t2 * 2.f * 3.14) + 1.0f) * ((x - 5.f) / 10.f),
                       (sin(t2 * 2.f * 3.14) + 1.0f) * ((y - 5.f) / 10.f),
                       (sin(t2 * 2.f * 3.14) + 1.0f) * ((z - 5.f) / 10.f), t1);
          }
        }
      }
    }
  }

  size_t totalParticles = 0;
  for (size_t j = 0; j < particleData.size(); ++j) {
    totalParticles += particleData[j].size();
  }
  std::cout << "Total particles " << totalParticles << std::endl;
  std::cout << "Avg Particles Per Step "
            << totalParticles / float(particleData.size()) << std::endl;
  std::cout << "Num steps " << particleData.size() << std::endl;
  ;

  float3 aabb[2] = {
      {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
       std::numeric_limits<float>::max()},
      {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
       -std::numeric_limits<float>::max()},
  };

  std::vector<float3> aabbs;

  std::cout << "Computing bounding box..." << std::endl;
  for (size_t j = 0; j < particleData.size(); ++j) {
    float3 thisAabb[2] = {
        {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
         std::numeric_limits<float>::max()},
        {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
         -std::numeric_limits<float>::max()},
    };

    for (size_t i = 0; i < particleData[j].size(); ++i) {
      aabb[0] = linalg::min(aabb[0], particleData[j][i].second.xyz());
      aabb[1] = linalg::max(aabb[1], particleData[j][i].second.xyz());

      thisAabb[0] = linalg::min(thisAabb[0], particleData[j][i].second.xyz());
      thisAabb[1] = linalg::max(thisAabb[1], particleData[j][i].second.xyz());
    }

    aabbs.push_back(thisAabb[0]);
    aabbs.push_back(thisAabb[1]);
  }
  std::cout << " - Done!" << std::endl;

  std::vector<float> camParams = program.get<std::vector<float>>("--camera");

  if (camParams.size() > 0) {
    lookFrom = float3(camParams[0], camParams[1], camParams[2]);
    lookAt = float3(camParams[3], camParams[4], camParams[5]);
    lookUp = float3(camParams[6], camParams[7], camParams[8]);
    cosFovy = camParams[9];
  } else {
    // set focus to aabb
    lookAt = (aabb[1] + aabb[0]) * .5f;
    lookFrom = aabb[1];

    if (synthetic) {
      lookAt = {0.f, 0.f, 0.f};
      lookFrom = {-3.f, 3.f, 3.f};
    }
  }

  bool camChanged = camParams.empty();

  // Now, we compute hilbert codes per-point
  std::cout << "Computing hilbert codes..." << std::endl;
  double beforeHilbert = getCurrentTime();
  for (size_t j = 0; j < particleData.size(); ++j) {
    std::for_each(std::execution::par_unseq, std::begin(particleData[j]),
                  std::end(particleData[j]), [&](auto &&i) {
                    float3 tmp =
                        (i.second.xyz() - aabb[0]) / (aabb[1] - aabb[0]);
                    tmp.x = tmp.x * (float)(1 << 16);
                    tmp.y = tmp.y * (float)(1 << 16);
                    tmp.z = tmp.z * (float)(1 << 16);
                    const bitmask_t coord[3] = {
                        bitmask_t(tmp.x), bitmask_t(tmp.y), bitmask_t(tmp.z)};
                    i.first = hilbert_c2i(3, 16, coord);
                  });
  }
  double afterHilbert = getCurrentTime();
  double hilbertTime = afterHilbert - beforeHilbert;
  std::cout << " - Done!" << std::endl;

  std::cout << "Sorting points along hilbert curve..." << std::endl;
  double beforeSort = getCurrentTime();
  for (size_t j = 0; j < particleData.size(); ++j) {
#ifdef _WIN32
    // not sure why this isn't working on windows currently.
    std::sort(particleData[j].begin(), particleData[j].end());
#else
    std::sort(std::execution::par_unseq, particleData[j].begin(),
              particleData[j].end());
#endif
  }
  double afterSort = getCurrentTime();
  double sortTime = afterSort - beforeSort;
  std::cout << " - Done!" << std::endl;

  // here just transferring to a vector we can actually use.
  maxNumParticles = 0;

  float minScalarValue = +1e20f;
  float maxScalarValue = -1e20f;

  double beforeValueRange = getCurrentTime();
  particles.resize(particleData.size());
  for (size_t j = 0; j < particleData.size(); ++j) {
    particles[j].resize(particleData[j].size());
    for (size_t i = 0; i < particles[j].size(); ++i) {
      particles[j][i] = particleData[j][i].second;
      minScalarValue = std::min(particles[j][i].w, minScalarValue);
      maxScalarValue = std::max(particles[j][i].w, maxScalarValue);
    }
    particleData[j].clear();
    maxNumParticles = std::max(maxNumParticles, particles[j].size());
  }
  double afterValueRange = getCurrentTime();
  double valueRangeTime = afterValueRange - beforeValueRange;

  // normalize attributes
  if (maxScalarValue > minScalarValue) {
    for (size_t j = 0; j < particles.size(); ++j) {
      for (size_t i = 0; i < particles[j].size(); ++i) {
        particles[j][i].w = (particles[j][i].w - minScalarValue) /
                            (maxScalarValue - minScalarValue);
      }
    }
  }

#ifndef HEADLESS
  gprtRequestWindow(fbSize.x, fbSize.y, "RT Point Clouds");
#endif
  gprtRequestRayTypeCount(2);

  GPRTContext context = gprtContextCreate();
  GPRTModule moduleCommon = gprtModuleCreate(context, deviceCodeCommon);
  GPRTModule moduleSplat = gprtModuleCreate(context, deviceCodeSplat);
  GPRTModule moduleRBF = gprtModuleCreate(context, deviceCodeRBF);
  GPRTModule moduleVoxel = gprtModuleCreate(context, deviceCodeVoxel);

  auto GenRBFBounds =
      gprtComputeCreate<ParticleData>(context, moduleCommon, "GenRBFBounds");
  auto AccumulateRBFBounds = gprtComputeCreate<RayGenData>(
      context, moduleCommon, "AccumulateRBFBounds");
  auto AverageRBFBounds =
      gprtComputeCreate<RayGenData>(context, moduleCommon, "AverageRBFBounds");
  auto CompositeGui =
      gprtComputeCreate<RayGenData>(context, moduleCommon, "CompositeGui");

  auto particleType = gprtGeomTypeCreate<ParticleData>(context, GPRT_AABBS);
  gprtGeomTypeSetIntersectionProg(particleType, 1, moduleSplat,
                                  "ParticleSplatIntersection");
  gprtGeomTypeSetAnyHitProg(particleType, 1, moduleSplat,
                            "ParticleSplatAnyHit");

  gprtGeomTypeSetIntersectionProg(particleType, 0, moduleRBF,
                                  "ParticleRBFIntersection");
  gprtGeomTypeSetAnyHitProg(particleType, 0, moduleRBF, "ParticleRBFAnyHit");

  GPRTMissOf<MissProgData> miss =
      gprtMissCreate<MissProgData>(context, moduleCommon, "miss");
  GPRTRayGenOf<RayGenData> ParticleSplatRayGen =
      gprtRayGenCreate<RayGenData>(context, moduleSplat, "ParticleSplatRayGen");

  GPRTRayGenOf<RayGenData> ParticleRBFRayGen =
      gprtRayGenCreate<RayGenData>(context, moduleRBF, "ParticleRBFRayGen");

  GPRTRayGenOf<RayGenData> ParticleVoxelRayGen =
      gprtRayGenCreate<RayGenData>(context, moduleVoxel, "ParticleVoxelRayGen");

  RayGenData raygenData = {};

  auto frameBuffer =
      gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);
  auto accumBuffer =
      gprtDeviceBufferCreate<float4>(context, fbSize.x * fbSize.y);
  auto imageBuffer =
      gprtDeviceBufferCreate<float4>(context, fbSize.x * fbSize.y);
  auto taaBuffer = gprtDeviceBufferCreate<float4>(context, fbSize.x * fbSize.y);
  auto taaPrevBuffer =
      gprtDeviceBufferCreate<float4>(context, fbSize.x * fbSize.y);
  auto imageTexture = gprtDeviceTextureCreate<float4>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R32G32B32A32_SFLOAT, fbSize.x,
      fbSize.y, 1, false, nullptr);
  // auto imageTexture = gprtDeviceTextureCreate<float4>(
  //     context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R32G32B32A32_SFLOAT, fbSize.x,
  //     fbSize.y, 1, false, nullptr);

  auto guiColorAttachment = gprtDeviceTextureCreate<uint32_t>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_SRGB, fbSize.x,
      fbSize.y, 1, false, nullptr);
  auto guiDepthAttachment = gprtDeviceTextureCreate<float>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_D32_SFLOAT, fbSize.x, fbSize.y,
      1, false, nullptr);
  gprtGuiSetRasterAttachments(context, guiColorAttachment, guiDepthAttachment);

  raygenData.fbSize = fbSize;

  // Blue noise
  std::vector<GPRTTextureOf<stbi_uc>> stbn(64);
  std::vector<gprt::Texture> stbnHandles(64);
  for (uint32_t i = 0; i < 64; ++i) {
    std::string path = "";
    path += STBN_DIR;
    path += "stbn_vec3_2Dx1D_128x128x64_";
    path += std::to_string(i);
    path += ".png";
    int texWidth, texHeight, texChannels;
    stbi_uc *pixels = stbi_load(path.c_str(), &texWidth, &texHeight,
                                &texChannels, STBI_rgb_alpha);

    stbn[i] = gprtDeviceTextureCreate<stbi_uc>(
        context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_UNORM, texWidth,
        texHeight, /*depth*/ 1,
        /* generate mipmaps */ true, pixels);
    stbnHandles[i] = gprtTextureGetHandle(stbn[i]);
  }
  auto stbnBuffer =
      gprtDeviceBufferCreate<gprt::Texture>(context, 64, stbnHandles.data());

  // Colormap for visualization
  auto colormap = gprtDeviceTextureCreate<uint8_t>(context, GPRT_IMAGE_TYPE_1D,
                                                   GPRT_FORMAT_R8G8B8A8_SRGB,
                                                   64, 1, 1, false, nullptr);

  auto radiusmap = gprtDeviceTextureCreate<uint8_t>(context, GPRT_IMAGE_TYPE_1D,
                                                    GPRT_FORMAT_R8G8B8A8_SRGB,
                                                    64, 1, 1, false, nullptr);

  auto densitymap = gprtDeviceTextureCreate<uint8_t>(
      context, GPRT_IMAGE_TYPE_1D, GPRT_FORMAT_R8G8B8A8_SRGB, 64, 1, 1, false,
      nullptr);

  auto sampler =
      gprtSamplerCreate(context, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR,
                        GPRT_FILTER_LINEAR, 1, GPRT_SAMPLER_ADDRESS_MODE_CLAMP);

  raygenData.frameBuffer = gprtBufferGetHandle(frameBuffer);
  raygenData.imageBuffer = gprtBufferGetHandle(imageBuffer);
  raygenData.accumBuffer = gprtBufferGetHandle(accumBuffer);
  raygenData.taaBuffer = gprtBufferGetHandle(taaBuffer);
  raygenData.taaPrevBuffer = gprtBufferGetHandle(taaPrevBuffer);
  raygenData.stbnBuffer = gprtBufferGetHandle(stbnBuffer);
  raygenData.exposure = 1.f;
  raygenData.gamma = 1.0f;
  raygenData.colormap = gprtTextureGetHandle(colormap);
  raygenData.radiusmap = gprtTextureGetHandle(radiusmap);
  raygenData.densitymap = gprtTextureGetHandle(densitymap);
  raygenData.colormapSampler = gprtSamplerGetHandle(sampler);
  raygenData.guiTexture = gprtTextureGetHandle(guiColorAttachment);
  raygenData.imageTexture = gprtTextureGetHandle(imageTexture);
  raygenData.globalAABBMin = aabb[0];
  raygenData.globalAABBMax = aabb[1];
  raygenData.disableColorCorrection = false;
  raygenData.disableBlueNoise = false;
  raygenData.disableTAA = true;
  // raygenData.rbfRadius = rbfRadius;

  MissProgData *missData = gprtMissGetParameters(miss);
  ini.get_vec3f("color0", missData->color0.x, missData->color0.y,
                missData->color0.z, 0.1f, 0.1f, 0.1f);
  ini.get_vec3f("color1", missData->color1.x, missData->color1.y,
                missData->color1.z, 0.0f, 0.0f, 0.0f);

  ini.get_uint32("particlesPerLeaf", particlesPerLeaf, 1);
  std::cout << "Particles per leaf " << particlesPerLeaf << std::endl;
  uint32_t particlesPerLeafArg = program.get<uint32_t>("--particles-per-leaf");
  // overwrites ini!
  if (particlesPerLeafArg > 0)
    particlesPerLeaf = particlesPerLeafArg;

  auto particleBuffer =
      gprtDeviceBufferCreate<float4>(context, maxNumParticles, nullptr);
  auto aabbBuffer = gprtDeviceBufferCreate<float3>(
      context,
      2 * ((maxNumParticles + particlesPerLeaf - 1) / particlesPerLeaf),
      nullptr);
  auto particleGeom = gprtGeomCreate<ParticleData>(context, particleType);
  gprtAABBsSetPositions(particleGeom, aabbBuffer,
                        ((maxNumParticles + particlesPerLeaf - 1) /
                         particlesPerLeaf) /* just one aabb */);

  ParticleData particleRecord{};

  // particleRecord.numParticles = particles[0].size();
  particleRecord.particlesPerLeaf = particlesPerLeaf;
  // particleRecord.rbfRadius = rbfRadius;
  particleRecord.aabbs = gprtBufferGetHandle(aabbBuffer);
  particleRecord.particles = gprtBufferGetHandle(particleBuffer);
  particleRecord.colormap = gprtTextureGetHandle(colormap);
  particleRecord.radiusmap = gprtTextureGetHandle(radiusmap);
  particleRecord.colormapSampler = gprtSamplerGetHandle(sampler);
  particleRecord.disableColorCorrection = false;

  // also assign particles to raygen
  raygenData.particles = gprtBufferGetHandle(particleBuffer);
  // raygenData.numParticles = particles[0].size();
  raygenData.particlesPerLeaf = particlesPerLeaf;

  gprtGeomSetParameters(particleGeom, &particleRecord);
  gprtComputeSetParameters(GenRBFBounds, &particleRecord);
  gprtRayGenSetParameters(ParticleSplatRayGen, &raygenData);
  gprtRayGenSetParameters(ParticleRBFRayGen, &raygenData);
  gprtRayGenSetParameters(ParticleVoxelRayGen, &raygenData);

  // Upload parameters
  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

  // For now, allocate a grid of voxels for raster mode
  auto voxelVolume = gprtDeviceBufferCreate<float4>(
      context,
      structuredGridResolution * structuredGridResolution *
          structuredGridResolution,
      nullptr);
  auto voxelVolumeCount = gprtDeviceBufferCreate<float>(
      context,
      structuredGridResolution * structuredGridResolution *
          structuredGridResolution,
      nullptr);
  gprtBufferClear(voxelVolume);
  gprtBufferClear(voxelVolumeCount);
  uint3 dims = uint3(structuredGridResolution, structuredGridResolution,
                     structuredGridResolution);
  raygenData.volume = gprtBufferGetHandle(voxelVolume);
  raygenData.volumeCount = gprtBufferGetHandle(voxelVolumeCount);
  raygenData.volumeDimensions = dims;

  // copy raygen params
  gprtRayGenSetParameters(ParticleSplatRayGen, &raygenData);
  gprtRayGenSetParameters(ParticleRBFRayGen, &raygenData);
  gprtRayGenSetParameters(ParticleVoxelRayGen, &raygenData);
  gprtBuildShaderBindingTable(context);

  std::string cmMarksStr, rmMarksStr, dmMarksStr;
  ini.get_string("colormap", cmMarksStr);
  ini.get_string("radiusmap", rmMarksStr);
  ini.get_string("densitymap", dmMarksStr);

  auto getMarks = [](std::string markStr) {
    std::vector<std::pair<float, float4>> marks;
    if (markStr.empty())
      return marks;
    markStr = markStr.substr(1, markStr.size() - 2); // remove quotes
    auto s = string_split(markStr, ';');
    for (auto m : s) {
      float p, x, y, z, w;
      sscanf(m.c_str(), "%f:{%f,%f,%f,%f}", &p, &x, &y, &z, &w);
      marks.push_back({p, {x, y, z, w}});
    }
    return marks;
  };

  auto cmMarks = getMarks(cmMarksStr);
  auto rmMarks = getMarks(rmMarksStr);
  auto dmMarks = getMarks(dmMarksStr);

  if (cmMarks.empty()) {
    cmMarks.push_back({0.f, {0.f, 0.f, 1.f, 1.f}});
    cmMarks.push_back({0.25f, {0.f, 0.972549f, 1.f, 1.f}});
    cmMarks.push_back({0.5f, {0.f, 1.f, 0.00784314f, 1.f}});
    cmMarks.push_back({0.75f, {0.996078f, 1.f, 0.f, 1.f}});
    cmMarks.push_back({1.f, {1.f, 0.f, 0.f, 1.f}});
  }

  if (rmMarks.empty()) {
    rmMarks.push_back({0.f, {1.f, 1.f, 1.f, 1.f}});
    rmMarks.push_back({1.f, {1.f, 1.f, 1.f, 1.f}});
  }

  if (dmMarks.empty()) {
    dmMarks.push_back({0.0f, {0.f, 0.f, 0.f, 0.f}});
    dmMarks.push_back({1.f, {1.f, 1.f, 1.f, 1.f}});
  }

  std::list<ImGG::Mark> cmMarksImGG, rmMarksImGG, dmMarksImGG;
  for (auto m : cmMarks)
    cmMarksImGG.push_back(ImGG::Mark(
        ImGG::RelativePosition(m.first),
        ImGG::ColorRGBA{m.second.x, m.second.y, m.second.z, m.second.w}));

  for (auto m : rmMarks)
    rmMarksImGG.push_back(ImGG::Mark(
        ImGG::RelativePosition(m.first),
        ImGG::ColorRGBA{m.second.x, m.second.y, m.second.z, m.second.w}));

  for (auto m : dmMarks)
    dmMarksImGG.push_back(ImGG::Mark(
        ImGG::RelativePosition(m.first),
        ImGG::ColorRGBA{m.second.x, m.second.y, m.second.z, m.second.w}));

  ImGG::GradientWidget colormapWidget{cmMarksImGG};
  ImGG::GradientWidget radiusmapWidget{rmMarksImGG};
  ImGG::GradientWidget densitymapWidget{dmMarksImGG};

  ImGG::Settings grayscaleWidgetSettings{};
  grayscaleWidgetSettings.flags =
      ImGG::Flag::NoColor | ImGG::Flag::NoColormapDropdown;

  bool majorantsOutOfDate = true;
  bool voxelized = false;
  bool firstFrame = true;
  double xpos = 0.f, ypos = 0.f;
  double lastxpos, lastypos;
  uint32_t accumID = 1;
  uint32_t frameID = 1;

  GPRTAccel particleAccel = gprtAABBAccelCreate(context, 1, &particleGeom);
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, &particleAccel);

  float diagonal = length(aabb[1] - aabb[0]);

  int previousParticleFrame = -1;
  float previousParticleRadius = ((synthetic) ? 0.05f : .01f) * diagonal;
  float radiusArg = program.get<float>("--radius");
  bool playAnimation = true;
  static bool disableBlueNoise = false;
  static bool disableTAA = true;
  std::stringstream frameStats;
  do {
    ImGuiIO &io = ImGui::GetIO();
    ImGui::NewFrame();

    static int particleFrame = 0;
    ini.get_int32("particleFrame", particleFrame);

    ImGui::SliderInt("Frame", &particleFrame, 0, particles.size() - 1);

    if (ImGui::Button("Play Animation")) {
      playAnimation = true;
    }
    if (ImGui::Button("Pause Animation")) {
      playAnimation = false;
    }

    if (playAnimation) {
      particleFrame++;
      if (particleFrame >= particles.size())
        particleFrame = 1;
      accumID = 1;
    }

    static float rbfRadius = previousParticleRadius;
    ini.get_float("rbfRadius", rbfRadius);
    // cmdline overrides ini!
    if (rbfRadius != previousParticleRadius && radiusArg > 0.f)
      rbfRadius = radiusArg;

    ImGui::DragFloat("Particle Radius", &rbfRadius, 0.0001f * diagonal,
                     .0001f * diagonal, 1.f * diagonal, "%.5f");

    auto make_8bit = [](const float f) -> uint32_t {
      return std::min(255, std::max(0, int(f * 256.f)));
    };

    if (colormapWidget.widget("Attribute Colormap") || firstFrame) {
      gprtTextureMap(colormap);
      uint8_t *ptr = gprtTextureGetPointer(colormap);
      for (uint32_t i = 0; i < 64; ++i) {
        auto result =
            colormapWidget.gradient().at(ImGG::RelativePosition(i / 63.f));
        ptr[i * 4 + 0] = make_8bit(pow(result.x, 1.f / 2.2f));
        ptr[i * 4 + 1] = make_8bit(pow(result.y, 1.f / 2.2f));
        ptr[i * 4 + 2] = make_8bit(pow(result.z, 1.f / 2.2f));
        ptr[i * 4 + 3] = make_8bit(result.w);
      }

      accumID = 1;
      voxelized = false;
      majorantsOutOfDate = true;
      gprtTextureUnmap(colormap);
    }

    bool radiusEdited = false;
    if (radiusmapWidget.widget("RBF Radius", grayscaleWidgetSettings) ||
        firstFrame) {
      radiusEdited = true;
      gprtTextureMap(radiusmap);
      uint8_t *ptr = gprtTextureGetPointer(radiusmap);
      for (uint32_t i = 0; i < 64; ++i) {
        auto result =
            radiusmapWidget.gradient().at(ImGG::RelativePosition(i / 63.f));
        ptr[i * 4 + 0] = make_8bit(result.x);
      }

      accumID = 1;
      voxelized = false;
      majorantsOutOfDate = true;
      gprtTextureUnmap(radiusmap);
    }

    bool densityEdited = false;
    if (densitymapWidget.widget("RBF Density", grayscaleWidgetSettings) ||
        firstFrame) {
      densityEdited = true;
      gprtTextureMap(densitymap);
      uint8_t *ptr = gprtTextureGetPointer(densitymap);
      for (uint32_t i = 0; i < 64; ++i) {
        auto result =
            densitymapWidget.gradient().at(ImGG::RelativePosition(i / 63.f));
        std::cout << "density " << result.x << std::endl;
        ptr[i * 4 + 0] = make_8bit(result.x);
      }

      accumID = 1;
      voxelized = false;
      majorantsOutOfDate = true;
      gprtTextureUnmap(densitymap);
    }

    static bool disableColorCorrection = false;
    ini.get_bool("disableColorCorrection", disableColorCorrection);
    if (ImGui::Checkbox("Disable color correction", &disableColorCorrection)) {
      particleRecord.disableColorCorrection = disableColorCorrection;
      gprtGeomSetParameters(particleGeom, &particleRecord);
      raygenData.disableColorCorrection = disableColorCorrection;
      voxelized = false;
      accumID = 1;
    }

    if (ImGui::Checkbox("Disable Blue Noise", &disableBlueNoise)) {
      raygenData.disableBlueNoise = disableBlueNoise;
      voxelized = false;
      accumID = 1;
    }

    if (ImGui::Checkbox("Disable Temporal Antialiasing", &disableTAA)) {
      raygenData.disableTAA = disableTAA;
      voxelized = false;
      accumID = 1;
    }

    static float exposure = 1.f;
    static float gamma = 1.0f;
    ini.get_float("exposure", exposure);
    ini.get_float("gamma", gamma);
    ImGui::DragFloat("Exposure", &exposure, 0.01f, 0.0f, 5.f);
    ImGui::DragFloat("Gamma", &gamma, 0.01f, 0.0f, 5.f);
    raygenData.exposure = exposure;
    raygenData.gamma = gamma;

    bool fovChanged =
        ImGui::DragFloat("Field of View", &cosFovy, .01f, 0.1f, 3.f);

    static int mode = 1;
    ini.get_int32("mode", mode);
    if (ImGui::RadioButton("AA-RBF (Ours)", &mode, 1))
      accumID = 1;
    if (ImGui::RadioButton("Splatting (Knoll 2019)", &mode, 0))
      accumID = 1;
    if (ImGui::RadioButton("Voxelized", &mode, 2))
      accumID = 1;

    float speed = .001f;
    lastxpos = xpos;
    lastypos = ypos;
    gprtGetCursorPos(context, &xpos, &ypos);
    if (firstFrame) {
      lastxpos = xpos;
      lastypos = ypos;
    }

    float dx = xpos - lastxpos;
    float dy = ypos - lastypos;

    int state = gprtGetMouseButton(context, GPRT_MOUSE_BUTTON_LEFT);
    int rstate = gprtGetMouseButton(context, GPRT_MOUSE_BUTTON_RIGHT);
    int mstate = gprtGetMouseButton(context, GPRT_MOUSE_BUTTON_MIDDLE);

    int w_state = gprtGetKey(context, GPRT_KEY_W);
    int c_state = gprtGetKey(context, GPRT_KEY_C);
    int b_state = gprtGetKey(context, GPRT_KEY_B);
    int x_state = gprtGetKey(context, GPRT_KEY_X);
    int y_state = gprtGetKey(context, GPRT_KEY_Y);
    int z_state = gprtGetKey(context, GPRT_KEY_Z);
    int ctrl_state = gprtGetKey(context, GPRT_KEY_LEFT_CONTROL);
    int left_shift = gprtGetKey(context, GPRT_KEY_LEFT_SHIFT);
    int right_shift = gprtGetKey(context, GPRT_KEY_RIGHT_SHIFT);
    int shift = left_shift || right_shift;

    // close window on Ctrl-W press
    if (w_state && ctrl_state) {
      break;
    }
    // close window on Ctrl-C press
    if (c_state && ctrl_state) {
      break;
    }
    // Shift-C prints the cam
    if (c_state && shift) {
      std::cout << "--camera " << lookFrom.x << ' ' << lookFrom.y << ' '
                << lookFrom.z << ' ' << lookAt.x << ' ' << lookAt.y << ' '
                << lookAt.z << ' ' << lookUp.x << ' ' << lookUp.y << ' '
                << lookUp.z << ' ' << cosFovy << '\n';
    }
    // Shift-B prints the aabb and center of _this_ time step
    if (b_state && shift) {
      std::cout << "AABB(" << particleFrame << "): " << aabbs[particleFrame * 2]
                << ',' << aabbs[particleFrame * 2 + 1] << ", center: "
                << (aabbs[particleFrame * 2] + aabbs[particleFrame * 2 + 1]) *
                       0.5f
                << ", diagonal: "
                << length(aabbs[particleFrame * 2] -
                          aabbs[particleFrame * 2 + 1])
                << '\n';
    }

    if (x_state) {
      lookUp = float3(1.f, 0.f, 0.f);
      if (left_shift)
        lookUp *= -1.f;
    }

    if (y_state) {
      lookUp = float3(0.f, 1.f, 0.f);
      if (left_shift)
        lookUp *= -1.f;
    }

    if (z_state) {
      lookUp = float3(0.f, 0.f, 1.f);
      if (left_shift)
        lookUp *= -1.f;
    }

    // If we click the mouse, we should rotate the camera
    if (state == GPRT_PRESS && !io.WantCaptureMouse || x_state || y_state ||
        z_state || fovChanged || firstFrame) {
      firstFrame = false;
      float4 position = {lookFrom.x, lookFrom.y, lookFrom.z, 1.f};
      float4 pivot = {lookAt.x, lookAt.y, lookAt.z, 1.0};
#ifndef M_PI
#define M_PI 3.1415926f
#endif

      // step 1 : Calculate the amount of rotation given the mouse movement.
      float deltaAngleX = (2 * M_PI / fbSize.x);
      float deltaAngleY = (M_PI / fbSize.y);
      float xAngle = -dx * deltaAngleX;
      float yAngle = -dy * deltaAngleY;

      // step 2: Rotate the camera around the pivot point on the first axis.
      float4x4 rotationMatrixX = rotation_matrix(rotation_quat(lookUp, xAngle));
      position = (mul(rotationMatrixX, (position - pivot))) + pivot;

      // step 3: Rotate the camera around the pivot point on the second axis.
      float3 lookRight = cross(lookUp, normalize(pivot - position).xyz());
      float4x4 rotationMatrixY =
          rotation_matrix(rotation_quat(lookRight, yAngle));
      lookFrom = ((mul(rotationMatrixY, (position - pivot))) + pivot).xyz();

      // ----------- compute variable values  ------------------
      float3 camera_pos = lookFrom;
      float3 camera_d00 = normalize(lookAt - lookFrom);
      float aspect = float(fbSize.x) / float(fbSize.y);
      float3 camera_ddu =
          cosFovy * aspect * normalize(cross(camera_d00, lookUp));
      float3 camera_ddv = cosFovy * normalize(cross(camera_ddu, camera_d00));
      camera_d00 -= 0.5f * camera_ddu;
      camera_d00 -= 0.5f * camera_ddv;

      // ----------- set variables  ----------------------------
      raygenData.camera.pos = camera_pos;
      raygenData.camera.dir_00 = camera_d00;
      raygenData.camera.dir_du = camera_ddu;
      raygenData.camera.dir_dv = camera_ddv;
      if (camParams.empty()) {
        auto error_pos =
            ini.get_vec3f("camera.pos", raygenData.camera.pos.x,
                          raygenData.camera.pos.y, raygenData.camera.pos.z);
        auto error_dir00 = ini.get_vec3f(
            "camera.dir_00", raygenData.camera.dir_00.x,
            raygenData.camera.dir_00.y, raygenData.camera.dir_00.z);
        auto error_dir_du = ini.get_vec3f(
            "camera.dir_du", raygenData.camera.dir_du.x,
            raygenData.camera.dir_du.y, raygenData.camera.dir_du.z);
        auto error_dir_dv = ini.get_vec3f(
            "camera.dir_dv", raygenData.camera.dir_dv.x,
            raygenData.camera.dir_dv.y, raygenData.camera.dir_dv.z);
      }

      accumID = 1;
    }

    if (rstate == GPRT_PRESS && !io.WantCaptureMouse) {
      float3 view_vec = lookFrom - lookAt;

      if (dy > 0.0) {
        view_vec.x *= 0.95;
        view_vec.y *= 0.95;
        view_vec.z *= 0.95;
      } else if (dy < 0.0) {
        view_vec.x *= 1.05;
        view_vec.y *= 1.05;
        view_vec.z *= 1.05;
      }

      lookFrom = lookAt + view_vec;

      raygenData.camera.pos = lookFrom;

      accumID = 1;
    }

    if (mstate == GPRT_PRESS && !io.WantCaptureMouse) {
      float4 position = {lookFrom.x, lookFrom.y, lookFrom.z, 1.f};
      float4 pivot = {lookAt.x, lookAt.y, lookAt.z, 1.0};
      float3 lookRight = cross(lookUp, normalize(pivot - position).xyz());

      float3 translation = lookRight * dx + lookUp * -dy;
      translation = translation * .001f * diagonal;

      lookFrom = lookFrom + translation;
      lookAt = lookAt + translation;

      // ----------- compute variable values  ------------------
      float3 camera_pos = lookFrom;
      float3 camera_d00 = normalize(lookAt - lookFrom);
      float aspect = float(fbSize.x) / float(fbSize.y);
      float3 camera_ddu =
          cosFovy * aspect * normalize(cross(camera_d00, lookUp));
      float3 camera_ddv = cosFovy * normalize(cross(camera_ddu, camera_d00));
      camera_d00 -= 0.5f * camera_ddu;
      camera_d00 -= 0.5f * camera_ddv;

      // ----------- set variables  ----------------------------
      raygenData.camera.pos = camera_pos;
      raygenData.camera.dir_00 = camera_d00;
      raygenData.camera.dir_du = camera_ddu;
      raygenData.camera.dir_dv = camera_ddv;
      accumID = 1;
    }

    static float sigma = 3.f;
    static float clampMaxCumulativeValue = 1.f;
    static float unit = previousParticleRadius * .1f;
    static float jitter = 1.f;
    ini.get_float("sigma", sigma);
    ini.get_float("clampMaxCumulativeValue", clampMaxCumulativeValue);
    ini.get_float("unit", unit);
    ini.get_float("jitter", jitter);
    if (ImGui::DragFloat("clamp max cumulative value", &clampMaxCumulativeValue,
                         1.f, 0.f, 5000.f)) {
      accumID = 1;
      majorantsOutOfDate = true;
      voxelized = false;
    }
    if (ImGui::DragFloat("gaussian sigma", &sigma, 1.f, 0.f, 100.f)) {
      accumID = 1;
      majorantsOutOfDate = true;
      voxelized = false;
    }
    if (ImGui::InputFloat("step size", &unit, 0.0f, 0.0f, "%.4f"))
      accumID = 1;
    if (ImGui::InputFloat("jitter", &jitter, 0.0f, 0.0f, "%.4f"))
      accumID = 1;

    static bool visualizeAttributes = true;
    ini.get_bool("visualizeAttributes", visualizeAttributes);
    if (ImGui::Checkbox("Visualize Attributes", &visualizeAttributes)) {
      accumID = 1;
      voxelized = false;
    }

    unit = std::max(unit, .0001f);
    static float azimuth = 0.f;
    static float elevation = 0.f;
    static float ambient = .5f;
    ini.get_float("light.azimuth", azimuth);
    ini.get_float("light.elevation", elevation);
    ini.get_float("light.ambient", ambient);

    if (ImGui::SliderFloat("azimuth", &azimuth, 0.f, 1.f))
      accumID = 1;
    if (ImGui::SliderFloat("elevation", &elevation, -1.f, 1.f))
      accumID = 1;
    if (ImGui::SliderFloat("ambient", &ambient, 0.f, 1.f))
      accumID = 1;
    ImGui::EndFrame();

    static bool showHeatmap = false;
    ini.get_bool("showHeatmap", showHeatmap);

    raygenData.clampMaxCumulativeValue = clampMaxCumulativeValue;
    raygenData.sigma = sigma;
    raygenData.unit = unit;
    raygenData.jitter = jitter;
    raygenData.visualizeAttributes = visualizeAttributes;
    raygenData.showHeatmap = showHeatmap;
    raygenData.light.ambient = ambient;
    raygenData.light.elevation = elevation;
    raygenData.light.azimuth = azimuth;

    particleRecord.clampMaxCumulativeValue = clampMaxCumulativeValue;
    particleRecord.sigma = sigma;
    particleRecord.visualizeAttributes = visualizeAttributes;

    bool forceRebuild = false;
    ;

    double accelBuildTime = 0.0;
    if (previousParticleFrame != particleFrame || forceRebuild) {

      if (synthetic) {
        azimuth = sin(gprtGetTime(context)) * .5 + .5;
        elevation = cos(gprtGetTime(context));
      }

      particleRecord.numParticles = particles[particleFrame].size();
      particleRecord.rbfRadius = previousParticleRadius;
      raygenData.numParticles = particles[particleFrame].size();
      raygenData.rbfRadius = previousParticleRadius;

      // Upload some particles
      gprtBufferMap(particleBuffer);
      float4 *particlePositions = gprtBufferGetPointer(particleBuffer);
      memcpy(particlePositions, particles[particleFrame].data(),
             sizeof(float4) * particles[particleFrame].size());
      gprtBufferUnmap(particleBuffer);

      gprtComputeSetParameters(GenRBFBounds, &particleRecord);
      gprtGeomSetParameters(particleGeom, &particleRecord);
      gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

      // Generate bounding boxes for those particles
      // note, unneeded boxes will be inactivated.
      gprtComputeLaunch1D(context, GenRBFBounds,
                          (maxNumParticles + (particlesPerLeaf - 1)) /
                              particlesPerLeaf);

      // Now we can build the tree
      double beforeAccelBuild = getCurrentTime();
      gprtAccelBuild(context, particleAccel,
                     GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE);
      size_t particleSize = gprtBufferGetSize(particleBuffer);
      size_t aabbSize = gprtBufferGetSize(aabbBuffer);
      size_t accelSize = gprtAccelGetSize(particleAccel);
      gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);
      double afterAccelBuild = getCurrentTime();
      accelBuildTime = afterAccelBuild - beforeAccelBuild;

      // Assign tree to raygen parameters
      raygenData.world = gprtAccelGetHandle(world);

      voxelized = false;
      majorantsOutOfDate = true;
      accumID = 1;
      previousParticleFrame = particleFrame;
    }

    double accelUpdateTime = 0.0;
    if (previousParticleRadius != rbfRadius || radiusEdited || densityEdited ||
        forceRebuild) {
      particleRecord.rbfRadius = rbfRadius;
      raygenData.rbfRadius = rbfRadius;
      gprtComputeSetParameters(GenRBFBounds, &particleRecord);
      gprtGeomSetParameters(particleGeom, &particleRecord);
      gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

      // Regenerate bounding boxes for new radius
      gprtComputeLaunch1D(context, GenRBFBounds,
                          (maxNumParticles + (particlesPerLeaf - 1)) /
                              particlesPerLeaf);

      // Now we can refit the tree
      double beforeAccelUpdate = getCurrentTime();
      gprtAccelUpdate(context, particleAccel);
      gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);
      double afterAccelUpdate = getCurrentTime();
      accelUpdateTime = afterAccelUpdate - beforeAccelUpdate;

      std::cout << "accel update time " << accelUpdateTime * 1000 << std::endl;

      // Assign tree to raygen parameters
      raygenData.world = gprtAccelGetHandle(world);

      voxelized = false;
      majorantsOutOfDate = true;
      accumID = 1;
      previousParticleRadius = rbfRadius;
    }

    // if we need to, revoxelize
    if (mode == 2 && !voxelized) {
      auto accumParams =
          gprtComputeGetParameters<RayGenData>(AccumulateRBFBounds);
      auto avgParams = gprtComputeGetParameters<RayGenData>(AverageRBFBounds);
      *accumParams = *avgParams = raygenData;
      gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
      gprtBufferClear(voxelVolume);
      gprtBufferClear(voxelVolumeCount);
      gprtComputeLaunch1D(context, AccumulateRBFBounds, particles[0].size());
      gprtComputeLaunch1D(context, AverageRBFBounds, dims.x * dims.y * dims.z);
      voxelized = true;
    }

    gprtTextureClear(guiDepthAttachment);
    gprtTextureClear(guiColorAttachment);
    gprtGuiRasterize(context);

    raygenData.accumID = accumID;
    raygenData.frameID = frameID;

    // copy raygen params
    gprtRayGenSetParameters(ParticleSplatRayGen, &raygenData);
    gprtRayGenSetParameters(ParticleRBFRayGen, &raygenData);
    gprtRayGenSetParameters(ParticleVoxelRayGen, &raygenData);

    gprtComputeSetParameters(CompositeGui, &raygenData);

    gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

    gprtBeginProfile(context);

    switch (mode) {
    case 0:
      gprtRayGenLaunch2D(context, ParticleSplatRayGen, fbSize.x, fbSize.y);
      break;
    case 1:
      gprtRayGenLaunch2D(context, ParticleRBFRayGen, fbSize.x, fbSize.y);
      break;
    case 2:
      gprtRayGenLaunch2D(context, ParticleVoxelRayGen, fbSize.x, fbSize.y);
      break;
    default:
      break;
    }

    double profile = gprtEndProfile(context) / 1e9;
    static double tavg = profile;
    tavg = 0.8 * tavg + 0.2 * profile;

    char title[1000];
    sprintf(title, "%.2f FPS", (1.0 / tavg));

    gprtBufferTextureCopy(context, imageBuffer, imageTexture, 0, 0, 0, 0, 0, 0,
                          fbSize.x, fbSize.y, 1);

    gprtComputeLaunch2D(context, CompositeGui, fbSize.x, fbSize.y);

    gprtSetWindowTitle(context, title);
    gprtBufferPresent(context, frameBuffer);

    accumID++;
    ;
    frameID++;
    ;

    gprtBufferCopy(context, taaBuffer, taaPrevBuffer, 0, 0,
                   fbSize.x * fbSize.y);

    // Clear .ini file so we don't read entries from it
    // during the next loop iteration!
    ini.clear();
  } while (!gprtWindowShouldClose(context));

  LOG("cleaning up ...");

  gprtBufferDestroy(particleBuffer);
  gprtBufferDestroy(aabbBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(ParticleSplatRayGen);
  gprtRayGenDestroy(ParticleRBFRayGen);
  gprtRayGenDestroy(ParticleVoxelRayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(particleAccel);
  gprtAccelDestroy(world);
  gprtGeomDestroy(particleGeom);
  gprtGeomTypeDestroy(particleType);
  gprtModuleDestroy(moduleCommon);
  gprtModuleDestroy(moduleSplat);
  gprtModuleDestroy(moduleRBF);
  gprtModuleDestroy(moduleVoxel);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
