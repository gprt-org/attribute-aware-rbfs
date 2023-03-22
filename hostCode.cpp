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

#include "importers/import_points.h"
#include "importers/import_arborx.h"
#include "importers/import_mmpld.h"
#include "IniFile.h"
#include <argparse/argparse.hpp>

// For parallel sorting of points along a hilbert curve
#include <execution>
#include <algorithm>
#include "hilbert.h"

#define LOG(message)                                           \
  std::cout << GPRT_TERMINAL_BLUE;                             \
  std::cout << "#gprt.sample(main): " << message << std::endl; \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                        \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                       \
  std::cout << "#gprt.sample(main): " << message << std::endl; \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram deviceCodeCommon;
extern GPRTProgram deviceCodeSplat;
extern GPRTProgram deviceCodeRBF;
extern GPRTProgram deviceCodeVoxel;

// initial image resolution
// const int2 fbSize = {1334, 574}; // teaser size
const int2 fbSize = {1024, 1024}; // teaser size
// const int2 fbSize = {1920, 1080};

// Initial camera parameters
float3 lookFrom = {3.5f, 3.5f, 3.5f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

// uint32_t particlesPerLeaf = 4;
uint32_t particlesPerLeaf = 16;
// float rbfRadius = .01f; //3.f;
// float rbfRadius = 3.f; // 3.f;
std::vector<std::vector<float4>> particles;
size_t maxNumParticles;

uint32_t structuredGridResolution = 64;
uint32_t ddaGridResolution = 64;

static std::vector<std::string> string_split(std::string s, char delim) {
  std::vector<std::string> result;
  std::istringstream stream(s);

  for (std::string token; std::getline(stream, token, delim); ) {
    result.push_back(token);
  }

  return result;
}

#include <iostream>
int main(int argc, char *argv[])
{
  argparse::ArgumentParser program("RT Point Clouds");

  program.add_argument("--dbscan")
      .help("A path to a DBScan dataset (ending in .arborx)")
      .default_value("");
  program.add_argument("--mmpld")
      .help("A path to a MMPLD dataset (ending in .mmpld)")
      .default_value("");
  program.add_argument("--points")
      .help("A path to our custom points dataset (ending in .points)")
      .default_value("");

  program.add_argument("--camera")
    .nargs(10)
    .help("posx, posy, posz, atx, aty, atz, upx, upy, upz, fovy")
    .default_value(std::vector<float>{})
    .scan<'g', float>();
 
  #ifdef HEADLESS
  program.add_argument("--orbit")
    .help("Number of spherical orbits for benchmark")
    .default_value(0)
    .scan<'i', int>();

  program.add_argument("--orbit-center")
    .nargs(3)
    .help("Orbit center vector (cx, cy, cz)")
    .default_value(std::vector{{1e20f,1e20f,1e20f}})
    .scan<'g', float>();

  program.add_argument("--orbit-up")
    .nargs(3)
    .help("Orbit up vector (upx, upy, upz)")
    .default_value(std::vector{{0.f,1.f,0.f}})
    .scan<'g', float>();

  program.add_argument("--orbit-radius")
    .help("Orbit radius")
    .default_value(-1.f)
    .scan<'g', float>();
  #endif

  try {
    program.parse_args(argc, argv);
  }
  catch (const std::runtime_error &err)
  {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }

  IniFile ini("viewer.ini");
  if (ini.good())
    std::cout << "Found viewer.ini\n";

  std::vector<std::vector<std::pair<uint64_t, float4>>> particleData;

  std::string dbscanPath = program.get<std::string>("--dbscan");
  std::string mmpldPath = program.get<std::string>("--mmpld");
  std::string pointsPath = program.get<std::string>("--points");
  if (dbscanPath != "")
    importArborX(dbscanPath, particleData);
  else if (mmpldPath != "")
    importMMPLD(mmpldPath, particleData);
  else if (pointsPath != "")
    importPoints(pointsPath, particleData);
  else
  {
    ddaGridResolution = 1;
    particleData.resize(100);
    for (uint32_t frame = 0; frame < particleData.size(); ++frame) {
      particleData[frame].resize(3);
      for (uint32_t i = 0; i < particleData[frame].size(); ++i)
      {
        float t1 = float(i) / float(particleData[frame].size());
        
        float t2 = float(frame) / float(particleData.size());

        particleData[frame][i].second = float4(
          (cos(t2 * 3.14) * .5 + .5) * sin(t1 * 2.f * 3.14f),
          (cos(t2 * 3.14) * .5 + .5) * cos(t1 * 2.f * 3.14f),
          0.f, t1);
      }
    }
  }

  float3 aabb[2] = {
      {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
       std::numeric_limits<float>::max()},
      {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
       -std::numeric_limits<float>::max()},
  };

  std::cout << "Computing bounding box..." << std::endl;
  for (size_t j = 0; j < particleData.size(); ++j)
  {
    for (size_t i = 0; i < particleData[j].size(); ++i)
    {
      aabb[0] = linalg::min(aabb[0], particleData[j][i].second.xyz());
      aabb[1] = linalg::max(aabb[1], particleData[j][i].second.xyz());
    }
  }
  std::cout << " - Done!" << std::endl;

  std::vector<float> camParams = program.get<std::vector<float>>("--camera");

  if (camParams.size() > 0) {
    lookFrom = float3(camParams[0], camParams[1], camParams[2]);
    lookAt = float3(camParams[3], camParams[4], camParams[5]);
    lookUp = float3(camParams[6], camParams[7], camParams[8]);
    cosFovy = camParams[9];
  } 
  else {
    // set focus to aabb
    lookAt = (aabb[1] + aabb[0]) * .5f;
    lookFrom = aabb[1];
  }

  bool camChanged = camParams.empty();

  #ifdef HEADLESS
  int orbitCount = program.get<int>("--orbit");
  std::vector<float> vOrbitCenter = program.get<std::vector<float>>("--orbit-center");
  std::vector<float> vOrbitUp = program.get<std::vector<float>>("--orbit-up");
  float orbitRadius = program.get<float>("--orbit-radius");
  float3 orbitCenter(vOrbitCenter[0], vOrbitCenter[1], vOrbitCenter[2]);
  float3 orbitUp(vOrbitUp[0], vOrbitUp[1], vOrbitUp[2]);

  std::vector<float3> orbitCameraPositions;
  size_t currentOrbitPos = 0;

  if (orbitCount != 0) {
      if (orbitRadius <= 0.f) {
          orbitRadius = length(aabb[1] - aabb[0]) / 2.f;
      }
      std::cout << "orbit radius: " << orbitRadius << "\n";
      if (orbitCenter == float3(1e20f)) {
          orbitCenter = (aabb[0] + aabb[1]) / 2.f;
      }
      std::cout << "orbit center: " << orbitCenter << "\n";
      orbitCameraPositions = generate_fibonacci_sphere(orbitCount, orbitRadius);

      std::cout << "starting pos " << orbitCenter + orbitCameraPositions[currentOrbitPos] << "\n";

      lookFrom = lookAt + orbitCameraPositions[currentOrbitPos];
      lookAt   = orbitCenter;
      lookUp   = orbitUp;
  }
  #endif

  // Now, we compute hilbert codes per-point
  std::cout << "Computing hilbert codes..." << std::endl;
  for (size_t j = 0; j < particleData.size(); ++j)
  {
    std::for_each(std::execution::par_unseq, std::begin(particleData[j]),
                  std::end(particleData[j]), [&](auto &&i)
                  {
      float3 tmp = (i.second.xyz() - aabb[0]) / (aabb[1] - aabb[0]);
      tmp.x = tmp.x * (float)(1 << 16);
      tmp.y = tmp.y * (float)(1 << 16);
      tmp.z = tmp.z * (float)(1 << 16);
      const bitmask_t coord[3] = {bitmask_t(tmp.x), bitmask_t(tmp.y), bitmask_t(tmp.z)};
      i.first = hilbert_c2i(3, 16, coord); });
  }
  std::cout << " - Done!" << std::endl;

  std::cout << "Sorting points along hilbert curve..." << std::endl;
  for (size_t j = 0; j < particleData.size(); ++j)
  {
    std::sort(std::execution::par_unseq, particleData[j].begin(), particleData[j].end());
  }
  std::cout << " - Done!" << std::endl;

  // here just transferring to a vector we can actually use.
  maxNumParticles = 0;

  float minScalarValue = +1e20f;
  float maxScalarValue = -1e20f;

  particles.resize(particleData.size());
  for (size_t j = 0; j < particleData.size(); ++j) {
    particles[j].resize(particleData[j].size());
    for (size_t i = 0; i < particles[j].size(); ++i){
      particles[j][i] = particleData[j][i].second;
      minScalarValue = std::min(particles[j][i].w, minScalarValue);
      maxScalarValue = std::max(particles[j][i].w, maxScalarValue);
    }
    particleData[j].clear();
    maxNumParticles = std::max(maxNumParticles, particles[j].size());
  }

  // normalize attributes
  if (maxScalarValue > minScalarValue) {
    for (size_t j = 0; j < particles.size(); ++j) {
      for (size_t i = 0; i < particles[j].size(); ++i){
        particles[j][i].w = (particles[j][i].w - minScalarValue) / (maxScalarValue - minScalarValue);
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

  auto GenParticles =
      gprtComputeCreate<ParticleData>(context, moduleCommon, "GenParticles");
  auto GenRBFBounds =
      gprtComputeCreate<ParticleData>(context, moduleCommon, "GenRBFBounds");
  auto AccumulateRBFBounds = gprtComputeCreate<RayGenData>(
      context, moduleCommon, "AccumulateRBFBounds");
  auto AverageRBFBounds =
      gprtComputeCreate<RayGenData>(context, moduleCommon, "AverageRBFBounds");
  auto MinMaxRBFBounds =
      gprtComputeCreate<RayGenData>(context, moduleCommon, "MinMaxRBFBounds");
  auto ClearMinMaxGrid =
      gprtComputeCreate<RayGenData>(context, moduleCommon, "ClearMinMaxGrid");
  auto ComputeMajorantGrid =
      gprtComputeCreate<RayGenData>(context, moduleCommon, "ComputeMajorantGrid");

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
  auto imageBuffer =
      gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);
  auto accumBuffer =
      gprtDeviceBufferCreate<float4>(context, fbSize.x * fbSize.y);
  #ifndef HEADLESS
  auto guiColorAttachment = gprtDeviceTextureCreate<uint32_t>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_SRGB, fbSize.x,
      fbSize.y, 1, false, nullptr);
  auto guiDepthAttachment = gprtDeviceTextureCreate<float>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_D32_SFLOAT, fbSize.x, fbSize.y,
      1, false, nullptr);
  gprtGuiSetRasterAttachments(context, guiColorAttachment, guiDepthAttachment);
  #endif

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
    stbi_uc *pixels = stbi_load(path.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

    stbn[i] = gprtDeviceTextureCreate<stbi_uc>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_UNORM, texWidth, texHeight, /*depth*/ 1,
      /* generate mipmaps */ true, pixels);
    stbnHandles[i] = gprtTextureGetHandle(stbn[i]);
  }
  auto stbnBuffer = gprtDeviceBufferCreate<gprt::Texture>(context, 64, stbnHandles.data());

  // Colormap for visualization
  auto colormap = gprtDeviceTextureCreate<uint8_t>(context, GPRT_IMAGE_TYPE_1D,
                                                    GPRT_FORMAT_R8G8B8A8_SRGB,
                                                    64, 1, 1, false, nullptr);

  auto radiusmap = gprtDeviceTextureCreate<uint8_t>(context, GPRT_IMAGE_TYPE_1D,
                                                      GPRT_FORMAT_R8G8B8A8_SRGB,
                                                      64, 1, 1, false, nullptr);
  
  auto densitymap = gprtDeviceTextureCreate<uint8_t>(context, GPRT_IMAGE_TYPE_1D,
                                                      GPRT_FORMAT_R8G8B8A8_SRGB,
                                                      64, 1, 1, false, nullptr);

  auto sampler =
      gprtSamplerCreate(context, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR,
                        GPRT_FILTER_LINEAR, 1, GPRT_SAMPLER_ADDRESS_MODE_CLAMP);

  RayGenData *splatRayGenData = gprtRayGenGetParameters(ParticleSplatRayGen);
  RayGenData *rbfRayGenData = gprtRayGenGetParameters(ParticleRBFRayGen);
  RayGenData *voxelRayGenData = gprtRayGenGetParameters(ParticleVoxelRayGen);

  raygenData.frameBuffer = gprtBufferGetHandle(frameBuffer);
  raygenData.imageBuffer = gprtBufferGetHandle(imageBuffer);
  raygenData.accumBuffer = gprtBufferGetHandle(accumBuffer);
  raygenData.stbnBuffer = gprtBufferGetHandle(stbnBuffer);
  raygenData.exposure = 1.f;
  raygenData.gamma = 1.0f;
  raygenData.spp = 1;
  raygenData.colormap = gprtTextureGetHandle(colormap);
  raygenData.radiusmap = gprtTextureGetHandle(radiusmap);
  raygenData.densitymap = gprtTextureGetHandle(densitymap);
  raygenData.colormapSampler = gprtSamplerGetHandle(sampler);
  #ifndef HEADLESS
  raygenData.guiTexture = gprtTextureGetHandle(guiColorAttachment);
  #endif
  raygenData.globalAABBMin = aabb[0];
  raygenData.globalAABBMax = aabb[1];
  raygenData.disableColorCorrection = false;
  raygenData.disableBlueNoise = false;
  // raygenData.rbfRadius = rbfRadius;

  MissProgData *missData = gprtMissGetParameters(miss);
  ini.get_vec3f("color0", missData->color0.x, missData->color0.y, missData->color0.z, 0.1f, 0.1f, 0.1f);
  ini.get_vec3f("color1", missData->color1.x, missData->color1.y, missData->color1.z, 0.0f, 0.0f, 0.0f);

  auto particleBuffer =
      gprtDeviceBufferCreate<float4>(context, maxNumParticles, nullptr);
  auto aabbBuffer =
      gprtDeviceBufferCreate<float3>(context,2 * ((maxNumParticles + particlesPerLeaf - 1) / particlesPerLeaf), nullptr);
  auto particleGeom = gprtGeomCreate<ParticleData>(context, particleType);
  gprtAABBsSetPositions(particleGeom, aabbBuffer,
                        ((maxNumParticles + particlesPerLeaf - 1) / particlesPerLeaf) /* just one aabb */);

  ParticleData *particleRecord = gprtGeomGetParameters(particleGeom);
  // particleRecord->numParticles = particles[0].size();
  particleRecord->particlesPerLeaf = particlesPerLeaf;
  // particleRecord->rbfRadius = rbfRadius;
  particleRecord->aabbs = gprtBufferGetHandle(aabbBuffer);
  particleRecord->particles = gprtBufferGetHandle(particleBuffer);
  particleRecord->colormap = gprtTextureGetHandle(colormap);
  particleRecord->radiusmap = gprtTextureGetHandle(radiusmap);
  particleRecord->colormapSampler = gprtSamplerGetHandle(sampler);
  particleRecord->disableColorCorrection = false;

  // also assign particles to raygen
  raygenData.particles = gprtBufferGetHandle(particleBuffer);
  // raygenData.numParticles = particles[0].size();
  raygenData.particlesPerLeaf = particlesPerLeaf;

  // same parameters go to these compute programs too
  ParticleData *genParticlesData = gprtComputeGetParameters(GenParticles);
  ParticleData *genRBFBoundsData = gprtComputeGetParameters(GenRBFBounds);
  *genParticlesData = *particleRecord;
  *genRBFBoundsData = *particleRecord;
  *splatRayGenData = *rbfRayGenData = *voxelRayGenData = raygenData;

  // Upload parameters
  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

  // For now, allocate a grid of voxels for raster mode
  auto voxelVolume = gprtDeviceBufferCreate<float4>(
      context, structuredGridResolution * structuredGridResolution * structuredGridResolution, nullptr);
  auto voxelVolumeCount = gprtDeviceBufferCreate<float>(
      context, structuredGridResolution * structuredGridResolution * structuredGridResolution, nullptr);
  gprtBufferClear(voxelVolume);
  gprtBufferClear(voxelVolumeCount);
  uint3 dims = uint3(structuredGridResolution, structuredGridResolution, structuredGridResolution);
  raygenData.volume = gprtBufferGetHandle(voxelVolume);
  raygenData.volumeCount = gprtBufferGetHandle(voxelVolumeCount);
  raygenData.volumeDimensions = dims;

  // Grid of voxels for DDA
  // minmax volume stores both min and max of RBF as well as min and max of
  // attributes
  auto minMaxVolume = gprtDeviceBufferCreate<float4>(
      context, ddaGridResolution * ddaGridResolution * ddaGridResolution, nullptr);
  auto majorantVolume = gprtDeviceBufferCreate<float>(
      context, ddaGridResolution * ddaGridResolution * ddaGridResolution, nullptr);
  raygenData.minMaxVolume = gprtBufferGetHandle(minMaxVolume);
  raygenData.majorants = gprtBufferGetHandle(majorantVolume);
  uint3 ddaDimensions = uint3(ddaGridResolution, ddaGridResolution, ddaGridResolution);
  ini.get_vec3ui("ddaDimensions", ddaDimensions.x, ddaDimensions.y, ddaDimensions.z);
  raygenData.ddaDimensions = ddaDimensions;

  // copy raygen params
  *splatRayGenData = *rbfRayGenData = *voxelRayGenData = raygenData;
  gprtBuildShaderBindingTable(context);

  std::string cmMarksStr, rmMarksStr, dmMarksStr;
  ini.get_string("colormap", cmMarksStr);
  ini.get_string("radiusmap", rmMarksStr);
  ini.get_string("densitymap", dmMarksStr);

  auto getMarks = [](std::string markStr) {
    std::vector<std::pair<float,float4>> marks;
    if (markStr.empty())
      return marks;
    markStr = markStr.substr(1,markStr.size()-2); //remove quotes
    auto s = string_split(markStr,';');
    for (auto m : s) {
      float p,x,y,z,w;
      sscanf(m.c_str(),"%f:{%f,%f,%f,%f}",&p,&x,&y,&z,&w);
      marks.push_back({p,{x,y,z,w}});
    }
    return marks;
  };

  auto cmMarks=getMarks(cmMarksStr);
  auto rmMarks=getMarks(rmMarksStr);
  auto dmMarks=getMarks(dmMarksStr);

  if (cmMarks.empty()) {
    cmMarks.push_back({0.f,{1.f,1.f,1.f,1.f}});
    cmMarks.push_back({1.f,{1.f,1.f,1.f,1.f}});
  }

  if (rmMarks.empty()) {
    rmMarks.push_back({0.f,{1.f,1.f,1.f,1.f}});
    rmMarks.push_back({1.f,{1.f,1.f,1.f,1.f}});
  }

  if (dmMarks.empty()) {
    dmMarks.push_back({0.f,{1.f,1.f,1.f,1.f}});
    dmMarks.push_back({1.f,{1.f,1.f,1.f,1.f}});
  }

  #ifdef HEADLESS
  std::sort(cmMarks.begin(),cmMarks.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  std::sort(rmMarks.begin(),rmMarks.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  std::sort(dmMarks.begin(),dmMarks.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  ColorMap colormapInterpol{cmMarks};
  ColorMap radiusmapInterpol{rmMarks};
  ColorMap densitymapInterpol{dmMarks};
  #else
  std::list<ImGG::Mark> cmMarksImGG, rmMarksImGG, dmMarksImGG;
  for (auto m : cmMarks)
    cmMarksImGG.push_back(ImGG::Mark(ImGG::RelativePosition(m.first),
                                     ImGG::ColorRGBA{m.second.x,m.second.y,m.second.z,m.second.w}));

  for (auto m : rmMarks)
    rmMarksImGG.push_back(ImGG::Mark(ImGG::RelativePosition(m.first),
                                     ImGG::ColorRGBA{m.second.x,m.second.y,m.second.z,m.second.w}));

  for (auto m : dmMarks)
    dmMarksImGG.push_back(ImGG::Mark(ImGG::RelativePosition(m.first),
                                     ImGG::ColorRGBA{m.second.x,m.second.y,m.second.z,m.second.w}));

  ImGG::GradientWidget colormapWidget{cmMarksImGG};
  ImGG::GradientWidget radiusmapWidget{rmMarksImGG};
  ImGG::GradientWidget densitymapWidget{dmMarksImGG};

  ImGG::Settings grayscaleWidgetSettings{};
  grayscaleWidgetSettings.flags = ImGG::Flag::NoColor | ImGG::Flag::NoColormapDropdown;
  #endif 

  bool majorantsOutOfDate = true;
  bool voxelized = false;
  bool firstFrame = true;
  double xpos = 0.f, ypos = 0.f;
  double lastxpos, lastypos;
  uint32_t frameID = 1;

  GPRTAccel particleAccel = gprtAABBAccelCreate(context, 1, &particleGeom);
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, &particleAccel);

  float diagonal = length(aabb[1] - aabb[0]);

  int previousParticleFrame = -1;
  float previousParticleRadius = 0.001f * diagonal;
  bool renderAnimation = false;
  do
  {
    #ifndef HEADLESS
    ImGuiIO &io = ImGui::GetIO();
    ImGui::NewFrame();
    #endif
    

    static int particleFrame = 0;
    #ifdef HEADLESS
    //
    #else
    ImGui::SliderInt("Frame", &particleFrame, 0, particles.size() - 1);

    if (ImGui::Button("Render Animation")) {
      renderAnimation = true;
      particleFrame = 0;
      frameID = 1;
    }

    if (renderAnimation) {
      std::string text = "Rendering frame " + std::to_string(particleFrame) + ", ";
      text += std::to_string(frameID) + " / 2"; 
      ImGui::Text(text.c_str());


      if (frameID == 2) {
        std::string numberStr = std::to_string(particleFrame);
        auto new_str = std::string(3 - std::min(std::size_t(3), numberStr.length()), '0') + numberStr;
        gprtBufferSaveImage(imageBuffer, fbSize.x, fbSize.y, std::string("./image" + new_str + ".png").c_str());

        particleFrame++;
        frameID = 1;
        if (particleFrame == particles.size()) {
          renderAnimation = false;
          particleFrame = 0;
        }
      }
    }
    #endif



    static float rbfRadius = previousParticleRadius;
    ini.get_float("rbfRadius", rbfRadius);
    #ifndef HEADLESS
    ImGui::DragFloat("Particle Radius", &rbfRadius, 0.0001f * diagonal, .0001f * diagonal, 1.f * diagonal, "%.5f");
    #endif

    auto make_8bit = [](const float f) -> uint32_t
    {
      return std::min(255, std::max(0, int(f * 256.f)));
    };
   
    #ifdef HEADLESS
    if (firstFrame)
    #else
    if (colormapWidget.widget("Attribute Colormap") || firstFrame)
    #endif
    {
      gprtTextureMap(colormap);
      uint8_t *ptr = gprtTextureGetPointer(colormap);
      for (uint32_t i = 0; i < 64; ++i)
      {
        #ifdef HEADLESS
        float4 result = colormapInterpol.at(i / 63.f);
        #else
        auto result = colormapWidget.gradient().at(
            ImGG::RelativePosition(i / 63.f)
        );
        #endif
        ptr[i * 4 + 0] = make_8bit(pow(result.x, 1.f / 2.2f));
        ptr[i * 4 + 1] = make_8bit(pow(result.y, 1.f / 2.2f));
        ptr[i * 4 + 2] = make_8bit(pow(result.z, 1.f / 2.2f));
        ptr[i * 4 + 3] = make_8bit(result.w);
      }

      frameID = 1;
      voxelized = false;
      majorantsOutOfDate = true;
      gprtTextureUnmap(colormap);
    }

    bool radiusEdited = false;
    #ifdef HEADLESS
    if (firstFrame)
    #else
    if (radiusmapWidget.widget("RBF Radius", grayscaleWidgetSettings) || firstFrame)
    #endif
    {
      radiusEdited = true;
      gprtTextureMap(radiusmap);
      uint8_t *ptr = gprtTextureGetPointer(radiusmap);
      for (uint32_t i = 0; i < 64; ++i)
      {
        #ifdef HEADLESS
        float4 result = radiusmapInterpol.at(i / 63.f);
        #else
        auto result = radiusmapWidget.gradient().at(
            ImGG::RelativePosition(i / 63.f)
        );
        #endif
        ptr[i * 4 + 0] = make_8bit(result.x);
      }

      frameID = 1;
      voxelized = false;
      majorantsOutOfDate = true;
      gprtTextureUnmap(radiusmap);
    }

    #ifdef HEADLESS
    if (firstFrame)
    #else
    if (densitymapWidget.widget("RBF Density", grayscaleWidgetSettings) || firstFrame)
    #endif
    {
      gprtTextureMap(densitymap);
      uint8_t *ptr = gprtTextureGetPointer(densitymap);
      for (uint32_t i = 0; i < 64; ++i)
      {
        #ifdef HEADLESS
        float4 result = densitymapInterpol.at(i / 63.f);
        #else
        auto result = densitymapWidget.gradient().at(
            ImGG::RelativePosition(i / 63.f)
        );
        #endif
        ptr[i * 4 + 0] = make_8bit(result.x);
      }

      frameID = 1;
      voxelized = false;
      majorantsOutOfDate = true;
      gprtTextureUnmap(densitymap);
    }

    static bool disableColorCorrection = false;
    ini.get_bool("disableColorCorrection", disableColorCorrection);
    #ifndef HEADLESS
    if (ImGui::Checkbox("Disable color correction", &disableColorCorrection)) {
      particleRecord->disableColorCorrection = disableColorCorrection;
      raygenData.disableColorCorrection = disableColorCorrection;
      voxelized = false;
      frameID = 1;
    }

    static bool disableBlueNoise = false;
    if (ImGui::Checkbox("Disable Blue Noise", &disableBlueNoise)) {
      raygenData.disableBlueNoise = disableBlueNoise;
      voxelized = false;
      frameID = 1;
    }

    if (ImGui::Button("Save screenshot"))
    {
      gprtBufferSaveImage(imageBuffer, fbSize.x, fbSize.y, "./screenshot.png");
    }
    #endif
    

    static float exposure = 1.f;
    static float gamma = 1.0f;
    static int spp = 1;
    ini.get_float("exposure", exposure);
    ini.get_float("gamma", gamma);
    ini.get_int32("spp", spp);
    #ifndef HEADLESS
    ImGui::DragInt("Samples", &spp, 1, 1, 32);
    ImGui::DragFloat("Exposure", &exposure, 0.01f, 0.0f, 5.f);
    ImGui::DragFloat("Gamma", &gamma, 0.01f, 0.0f, 5.f);
    #endif
    raygenData.exposure = exposure;
    raygenData.gamma = gamma;
    raygenData.spp = spp;

    #ifdef HEADLESS
    bool fovChanged = firstFrame;
    #else
    bool fovChanged = ImGui::DragFloat("Field of View", &cosFovy, .01f, 0.1f, 3.f) ;
    #endif

    static int mode = 1;
    ini.get_int32("mode", mode);
    #ifdef HEADLESS
    //
    #else
    if (ImGui::RadioButton("Splatting", &mode, 0))
      frameID = 1;
    if (ImGui::RadioButton("RBF Query", &mode, 1))
      frameID = 1;
    if (ImGui::RadioButton("Voxelized", &mode, 2))
      frameID = 1;
    #endif



    float speed = .001f;
    lastxpos = xpos;
    lastypos = ypos;
    gprtGetCursorPos(context, &xpos, &ypos);
    if (firstFrame)
    {
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
    int x_state = gprtGetKey(context, GPRT_KEY_X);
    int y_state = gprtGetKey(context, GPRT_KEY_Y);
    int z_state = gprtGetKey(context, GPRT_KEY_Z);
    int i_state = gprtGetKey(context, GPRT_KEY_I);
    int ctrl_state  = gprtGetKey(context, GPRT_KEY_LEFT_CONTROL);
    int left_shift  = gprtGetKey(context, GPRT_KEY_LEFT_SHIFT);
    int right_shift = gprtGetKey(context, GPRT_KEY_RIGHT_SHIFT);
    int shift = left_shift || right_shift;

    // close window on Ctrl-W press
    if (w_state && ctrl_state)
    {
      break;
    }
    // close window on Ctrl-C press
    if (c_state && ctrl_state)
    {
      break;
    }
    // Shift-C prints the cam
    if (c_state && shift)
    {
      std::cout << "--camera " << lookFrom.x << ' ' << lookFrom.y << ' ' << lookFrom.z << ' '
                               << lookAt.x << ' ' << lookAt.y << ' ' << lookAt.z << ' '
                               << lookUp.x << ' ' << lookUp.y << ' ' << lookUp.z << ' '
                               << cosFovy << '\n';
    }


    if (x_state) {
      lookUp = float3(1.f, 0.f, 0.f);
      if (left_shift) lookUp *= -1.f;
    }

    if (y_state) {
      lookUp = float3(0.f, 1.f, 0.f);
      if (left_shift) lookUp *= -1.f;
    }

    if (z_state) {
      lookUp = float3(0.f, 0.f, 1.f);
      if (left_shift) lookUp *= -1.f;
    }

    #ifdef HEADLESS
    if (fovChanged || firstFrame)
    #else
    // If we click the mouse, we should rotate the camera
    if (state == GPRT_PRESS && !io.WantCaptureMouse || x_state || y_state || z_state || fovChanged || firstFrame)
    #endif
    {
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
        #ifdef HEADLESS
        if (orbitCount == 0) {
        #endif

        auto error_pos =
        ini.get_vec3f("camera.pos", raygenData.camera.pos.x,
                                    raygenData.camera.pos.y,
                                    raygenData.camera.pos.z);
        auto error_dir00 =
        ini.get_vec3f("camera.dir_00", raygenData.camera.dir_00.x,
                                       raygenData.camera.dir_00.y,
                                       raygenData.camera.dir_00.z);
        auto error_dir_du =
        ini.get_vec3f("camera.dir_du", raygenData.camera.dir_du.x,
                                       raygenData.camera.dir_du.y,
                                       raygenData.camera.dir_du.z);
        auto error_dir_dv =
        ini.get_vec3f("camera.dir_dv", raygenData.camera.dir_dv.x,
                                       raygenData.camera.dir_dv.y,
                                       raygenData.camera.dir_dv.z);

        #ifdef HEADLESS
        if (error_pos    == IniFile::Ok &&
            error_dir00  == IniFile::Ok &&
            error_dir_du == IniFile::Ok &&
            error_dir_dv == IniFile::Ok)
          camChanged = false; // don't recompute!
        #endif

        #ifdef HEADLESS
        }
        #endif
      }

      frameID = 1;
    }

    #ifndef HEADLESS
    if (rstate == GPRT_PRESS && !io.WantCaptureMouse)
    {
      float3 view_vec = lookFrom - lookAt;

      if (dy > 0.0)
      {
        view_vec.x *= 0.95;
        view_vec.y *= 0.95;
        view_vec.z *= 0.95;
      }
      else if (dy < 0.0)
      {
        view_vec.x *= 1.05;
        view_vec.y *= 1.05;
        view_vec.z *= 1.05;
      }

      lookFrom = lookAt + view_vec;

      raygenData.camera.pos = lookFrom;

      frameID = 1;
    }
    #endif

    #ifndef HEADLESS
    if (mstate == GPRT_PRESS && !io.WantCaptureMouse)
    #else
    if (camChanged)
    #endif
    {
    std::cout << lookAt << '\n';
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
     
      #if HEADLESS
      camChanged = false;
      #endif

      frameID = 1;
    }

    static float sigma = 3.f;
    static float clampMaxCumulativeValue = 1.f;
    static float unit = previousParticleRadius;
    ini.get_float("sigma", sigma);
    ini.get_float("clampMaxCumulativeValue", sigma);
    ini.get_float("unit", unit);
    #ifndef HEADLESS
    if (ImGui::DragFloat("clamp max cumulative value",
                           &clampMaxCumulativeValue, 1.f, 0.f, 5000.f))
    {
      frameID = 1;
      majorantsOutOfDate = true;
      voxelized = false;
    }
    if (ImGui::DragFloat("gaussian sigma",
                           &sigma, 1.f, 0.f, 100.f))
    {
      frameID = 1;
      majorantsOutOfDate = true;
      voxelized = false;
    }
    if (ImGui::InputFloat("delta tracking unit value", &unit))
      frameID = 1;
    #endif

    static bool visualizeAttributes = true;
    ini.get_bool("visualizeAttributes", visualizeAttributes);
    #ifndef HEADLESS
    if (ImGui::Checkbox("Visualize Attributes", &visualizeAttributes))
    {
      frameID = 1;
      // majorantsOutOfDate = true; // might do this later...
      voxelized = false;
    }
    #endif

    unit = std::max(unit, .001f);
    static float azimuth = 0.f;
    static float elevation = 0.f;
    static float ambient = .5f;
    ini.get_float("light.azimuth", azimuth);
    ini.get_float("light.elevation", elevation);
    ini.get_float("light.ambient", ambient);

    #ifdef HEADLESS
    //
    #else
    if (ImGui::SliderFloat("azimuth", &azimuth, 0.f, 1.f))
      frameID = 1;
    if (ImGui::SliderFloat("elevation", &elevation, -1.f, 1.f))
      frameID = 1;
    if (ImGui::SliderFloat("ambient", &ambient, 0.f, 1.f))
      frameID = 1;
    ImGui::EndFrame();
    #endif

    static bool useDDA = true;
    ini.get_bool("useDDA", useDDA);

    static bool showHeatmap = false;
    ini.get_bool("showHeatmap", showHeatmap);

    raygenData.clampMaxCumulativeValue = clampMaxCumulativeValue;
    raygenData.sigma = sigma;
    raygenData.unit = unit;
    raygenData.visualizeAttributes = visualizeAttributes;
    raygenData.useDDA = useDDA;
    raygenData.showHeatmap = showHeatmap;
    raygenData.light.ambient = ambient;
    raygenData.light.elevation = elevation;
    raygenData.light.azimuth = azimuth;

    particleRecord->clampMaxCumulativeValue = clampMaxCumulativeValue;
    particleRecord->sigma = sigma;
    particleRecord->visualizeAttributes = visualizeAttributes;

    // Shift-I prints an ini-file for the current program state
    if (i_state && shift)
    {
      auto marksToString = [](const auto marks) {
        std::stringstream stream;
        stream << '\"';
        size_t i=0;
        for (auto m : marks) {
          stream << m.position.get() << ':';
          stream << '{'
                 << m.color.x << ',' << m.color.y << ',' << m.color.z << ',' << m.color.w
                 << '}';
          if (i < marks.size()-1) stream << ';';
          i++;
        }
        stream << '\"';
        return stream.str();
      };
      #ifndef HEADLESS
      auto cmMarks = colormapWidget.gradient().get_marks();
      auto rmMarks = radiusmapWidget.gradient().get_marks();
      auto dmMarks = densitymapWidget.gradient().get_marks();
      #endif

      std::cout << "\n\n\n\nPut this in $PWD/viewer.ini:\n";
      std::cout << "[RayGenData]\n";
      std::cout << "rbfRadius=" << raygenData.rbfRadius << '\n';
      std::cout << "clampMaxCumulativeValue=" << raygenData.clampMaxCumulativeValue << '\n';
      std::cout << "sigma=" << raygenData.sigma << '\n';
      std::cout << "power=" << raygenData.power << '\n';
      std::cout << "ddaDimensions=" << raygenData.ddaDimensions << '\n';
      std::cout << "visualizeAttributes=" << raygenData.visualizeAttributes << '\n';
      std::cout << "useDDA=" << raygenData.useDDA << '\n';
      std::cout << "showHeatmap=" << raygenData.showHeatmap << '\n';
      std::cout << "disableColorCorrection=" << raygenData.disableColorCorrection << '\n';
      std::cout << "light.azimuth=" << raygenData.light.azimuth << '\n';
      std::cout << "light.elevation=" << raygenData.light.elevation << '\n';
      std::cout << "light.ambient=" << raygenData.light.ambient << '\n';
      std::cout << "camera.pos=" << raygenData.camera.pos << '\n';
      std::cout << "camera.dir_00=" << raygenData.camera.dir_00 << '\n';
      std::cout << "camera.dir_du=" << raygenData.camera.dir_du << '\n';
      std::cout << "camera.dir_dv=" << raygenData.camera.dir_dv << '\n';
      std::cout << "spp=" << raygenData.spp << '\n';
      std::cout << "exposure=" << raygenData.exposure << '\n';
      std::cout << "gamma=" << raygenData.gamma << '\n';
      std::cout << "\n[MissProgData]\n";
      std::cout << "color0=" << missData->color0 << '\n';
      std::cout << "color1=" << missData->color1 << '\n';
      std::cout << "\n[Misc.]\n";
      std::cout << "mode=" << mode << '\n';
      #ifndef HEADLESS
      std::cout << "colormap=" << marksToString(cmMarks) << '\n';
      std::cout << "radiusmap=" << marksToString(rmMarks) << '\n';
      std::cout << "densitymap=" << marksToString(dmMarks) << '\n';
      #endif
      std::cout << std::flush;
    }

    if (previousParticleFrame != particleFrame) {    
      std::cout<<"Num particles "<< particles[particleFrame].size();
      particleRecord->numParticles = particles[particleFrame].size();
      particleRecord->rbfRadius = previousParticleRadius;
      raygenData.numParticles = particles[particleFrame].size();
      raygenData.rbfRadius = previousParticleRadius;

      // Upload some particles
      gprtBufferMap(particleBuffer);
      float4 *particlePositions = gprtBufferGetPointer(particleBuffer);
      memcpy(particlePositions, particles[particleFrame].data(), sizeof(float4) * particles[particleFrame].size());
      gprtBufferUnmap(particleBuffer);

      *genRBFBoundsData = *particleRecord;
      gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

      // Generate bounding boxes for those particles
      // note, unneeded boxes will be inactivated.
      gprtComputeLaunch1D(context, GenRBFBounds, (maxNumParticles + (particlesPerLeaf - 1)) / particlesPerLeaf);

      // Now we can build the tree
      gprtAccelBuild(context, particleAccel, GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE);
      gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

      // Assign tree to raygen parameters
      raygenData.world = gprtAccelGetHandle(world);

      // compute minmax ranges
      {
        auto params = gprtComputeGetParameters<RayGenData>(MinMaxRBFBounds);
        *params = raygenData;
        gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

        uint64_t numVoxels = ddaGridResolution * ddaGridResolution * ddaGridResolution;
        gprtBufferClear(minMaxVolume);
        gprtComputeLaunch1D(context, MinMaxRBFBounds, (particles[particleFrame].size() + (particlesPerLeaf - 1)) / particlesPerLeaf);
        std::cout << "- Done!" << std::endl;
      }
      
      voxelized = false;
      majorantsOutOfDate = true;
      frameID = 1;
      previousParticleFrame = particleFrame;
    }

    if (previousParticleRadius != rbfRadius || radiusEdited) {
      particleRecord->rbfRadius = rbfRadius;
      raygenData.rbfRadius = rbfRadius;
      *genRBFBoundsData = *particleRecord;
      gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

      // Regenerate bounding boxes for new radius
      gprtComputeLaunch1D(context, GenRBFBounds, (maxNumParticles + (particlesPerLeaf - 1)) / particlesPerLeaf);

      // Now we can refit the tree
      gprtAccelUpdate(context, particleAccel);
      gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

      // Assign tree to raygen parameters
      raygenData.world = gprtAccelGetHandle(world);

      // compute minmax ranges
      {
        std::cout << "Computing minmax ranges" << std::endl;
        // auto params1 = gprtComputeGetParameters<RayGenData>(ClearMinMaxGrid);
        auto params = gprtComputeGetParameters<RayGenData>(MinMaxRBFBounds);
        *params = raygenData;
        gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

        uint64_t numVoxels = ddaGridResolution * ddaGridResolution * ddaGridResolution;
        // gprtComputeLaunch1D(context, ClearMinMaxGrid, numVoxels);
        gprtBufferClear(minMaxVolume);
        gprtComputeLaunch1D(context, MinMaxRBFBounds, (particles[particleFrame].size() + (particlesPerLeaf - 1)) / particlesPerLeaf);
        std::cout << "- Done!" << std::endl;
      }
      
      voxelized = false;
      majorantsOutOfDate = true;
      frameID = 1;
      previousParticleRadius = rbfRadius;
    }

    // if we need to, revoxelize
    if (mode == 2 && !voxelized)
    {
      auto accumParams =
          gprtComputeGetParameters<RayGenData>(AccumulateRBFBounds);
      auto avgParams = gprtComputeGetParameters<RayGenData>(AverageRBFBounds);
      *accumParams = *avgParams = raygenData;
      gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
      gprtBufferClear(voxelVolume);
      gprtBufferClear(voxelVolumeCount);
      gprtComputeLaunch1D(context, AccumulateRBFBounds, particles[0].size());
      gprtComputeLaunch1D(context, AverageRBFBounds,
                          dims.x * dims.y * dims.z);
      voxelized = true;
    }

    // if we need to, recompute majorants
    if (majorantsOutOfDate)
    {

      gprtBufferClear(majorantVolume);

      auto params = gprtComputeGetParameters<RayGenData>(ComputeMajorantGrid);
      *params = raygenData;
      gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
      uint64_t numVoxels = ddaGridResolution * ddaGridResolution * ddaGridResolution;
      gprtComputeLaunch1D(context, ComputeMajorantGrid, numVoxels);
      majorantsOutOfDate = false;
    }

    #ifndef HEADLESS
    gprtTextureClear(guiDepthAttachment);
    gprtTextureClear(guiColorAttachment);
    gprtGuiRasterize(context);
    #endif

    gprtBeginProfile(context);

    raygenData.frameID = frameID;

    // copy raygen params
    *splatRayGenData = *rbfRayGenData = *voxelRayGenData = raygenData;
    gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

    switch (mode)
    {
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
    tavg = 0.8*tavg + 0.2*profile;

    char title[1000];
    sprintf(title,"%.2f FPS",(1.0/tavg));

    #ifdef HEADLESS
    char fileName[1000];
    if (orbitCount > 0)
      sprintf(fileName,"./screenshot-%i.png",(int)currentOrbitPos);
    else
      sprintf(fileName,"./screenshot.png");
    printf("%s\r\n", title);
    gprtBufferSaveImage(imageBuffer, fbSize.x, fbSize.y, fileName);
    #else
    gprtSetWindowTitle(context, title);
    gprtBufferPresent(context, frameBuffer);
    #endif

    #ifdef HEADLESS
    if (orbitCount > 0) {
      ++currentOrbitPos;
      float3 nextPos = orbitCenter + orbitCameraPositions[currentOrbitPos];
      nextPos.z = std::abs(nextPos.z);
      std::cout << "Advance to orbit #" << currentOrbitPos << "\nOrbit pos " << nextPos << "\n";

      lookFrom = nextPos;
      lookAt   = orbitCenter;
      lookUp   = orbitUp;
      camChanged = true;
    }
    #endif

    frameID ++;;

    // Clear .ini file so we don't read entries from it
    // during the next loop iteration!
    ini.clear();
  }
  #ifdef HEADLESS
  while (currentOrbitPos < orbitCount);
  #else
  while (!gprtWindowShouldClose(context));
  #endif

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
