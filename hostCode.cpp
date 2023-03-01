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

// our shared data structures between host and device
#include "sharedCode.h"

#include "imgui.h"
#include <imgui_gradient/imgui_gradient.hpp>

#include <fstream>

#include "importers/arborx.h"
#include <argparse/argparse.hpp>

// For parallel sorting of points along a hilbert curve
#include <execution>
#include <algorithm>
#include "hilbert.h"

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
const int2 fbSize = {1000, 1000};

// Initial camera parameters
float3 lookFrom = {3.5f, 3.5f, 3.5f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

uint32_t particlesPerLeaf = 8;
float rbfRadius = .1f;
std::vector<float4> particles;

uint32_t structuredGridResolution = 256;
uint32_t ddaGridResolution = 256;

#include <iostream>
int main(int argc, char *argv[]) { 
  argparse::ArgumentParser program("RT Point Clouds");

  program.add_argument("--dbscan")
    .help("A path to a DBScan dataset (ending in .arborx)")
    .default_value("");

  try {
    program.parse_args(argc, argv);
  }
  catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }

  std::vector<std::pair<uint64_t, float4>> particleData;

  std::string dbscanPath = program.get<std::string>("--dbscan");
  if (dbscanPath != "") importArborX(dbscanPath, particleData);

  else {
    particleData.resize(10000);
    for (uint32_t i = 0; i < particleData.size(); ++i) {
      float t = float(i) / float(particleData.size());
      particleData[i].second = float4(t * sin(t * 256.f * 3.14f),  
                                      t * cos(t * 256.f * 3.14f), 
                                      0.f, t);
    }
  }

  float3 aabb[2] = {
      {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
       std::numeric_limits<float>::max()},
      {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
       -std::numeric_limits<float>::max()},
  };

  std::cout<<"Computing bounding box..."<<std::endl;

  for (size_t i = 0; i < particleData.size(); ++i) {    
    aabb[0] = linalg::min(aabb[0], particleData[i].second.xyz() - rbfRadius);
    aabb[1] = linalg::max(aabb[1], particleData[i].second.xyz() + rbfRadius);
  }
  std::cout<<" - Done!"<<std::endl;

  // set focus to aabb
  lookAt = (aabb[1] + aabb[0]) * .5f;
  
  // Now, we compute hilbert codes per-point
  std::cout<<"Computing hilbert codes..."<<std::endl;
  std::for_each(std::execution::par_unseq, std::begin(particleData), 
    std::end(particleData), [&](auto &&i) 
  {
    float3 tmp = (i.second.xyz() - aabb[0]) / (aabb[1] - aabb[0]);
    tmp.x = tmp.x * (float)(1 << 16);
    tmp.y = tmp.y * (float)(1 << 16);
    tmp.z = tmp.z * (float)(1 << 16);
    const bitmask_t coord[3] = {bitmask_t(tmp.x), bitmask_t(tmp.y), bitmask_t(tmp.z)};
    i.first = hilbert_c2i(3, 16, coord);
  });
  
  std::cout<<" - Done!"<<std::endl;

  std::cout<<"Sorting points along hilbert curve..."<<std::endl;  
  std::sort(std::execution::par_unseq, particleData.begin(), particleData.end());
  std::cout<<" - Done!"<<std::endl;

  // here just transferring to a vector we can actually use.
  particles.resize(particleData.size());
  for (size_t i = 0; i < particles.size(); ++i) particles[i] = particleData[i].second;
  particleData.clear();

  gprtRequestWindow(fbSize.x, fbSize.y, "RT Point Clouds");
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
  auto accumBuffer =
      gprtDeviceBufferCreate<float4>(context, fbSize.x * fbSize.y);
  auto guiColorAttachment = gprtDeviceTextureCreate<uint32_t>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_SRGB, fbSize.x,
      fbSize.y, 1, false, nullptr);
  auto guiDepthAttachment = gprtDeviceTextureCreate<float>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_D32_SFLOAT, fbSize.x, fbSize.y,
      1, false, nullptr);
  gprtGuiSetRasterAttachments(context, guiColorAttachment, guiDepthAttachment);

  // Colormap for visualization
  auto colormap = gprtDeviceTextureCreate<uint32_t>(context, GPRT_IMAGE_TYPE_1D,
                                                    GPRT_FORMAT_R8G8B8A8_SRGB,
                                                    256, 1, 1, false, nullptr);

  auto sampler =
      gprtSamplerCreate(context, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR,
                        GPRT_FILTER_LINEAR, 1, GPRT_SAMPLER_ADDRESS_MODE_CLAMP);

  RayGenData *splatRayGenData = gprtRayGenGetParameters(ParticleSplatRayGen);
  RayGenData *rbfRayGenData = gprtRayGenGetParameters(ParticleRBFRayGen);
  RayGenData *voxelRayGenData = gprtRayGenGetParameters(ParticleVoxelRayGen);

  raygenData.frameBuffer = gprtBufferGetHandle(frameBuffer);
  raygenData.accumBuffer = gprtBufferGetHandle(accumBuffer);
  raygenData.colormap = gprtTextureGetHandle(colormap);
  raygenData.colormapSampler = gprtSamplerGetHandle(sampler);
  raygenData.guiTexture = gprtTextureGetHandle(guiColorAttachment);
  raygenData.globalAABBMin = aabb[0];
  raygenData.globalAABBMax = aabb[1];
  raygenData.rbfRadius = rbfRadius;

  MissProgData *missData = gprtMissGetParameters(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  auto particleBuffer =
      gprtDeviceBufferCreate<float4>(context, particles.size(), nullptr);
  auto aabbBuffer =
      gprtDeviceBufferCreate<float3>(context, (2 * particles.size()) / particlesPerLeaf, nullptr);
  auto particleGeom = gprtGeomCreate<ParticleData>(context, particleType);
  gprtAABBsSetPositions(particleGeom, aabbBuffer,
                        particles.size() / particlesPerLeaf/* just one aabb */);

  ParticleData *particleRecord = gprtGeomGetParameters(particleGeom);
  particleRecord->numParticles = particles.size();
  particleRecord->particlesPerLeaf = particlesPerLeaf;
  particleRecord->rbfRadius = rbfRadius;
  particleRecord->aabbs = gprtBufferGetHandle(aabbBuffer);
  particleRecord->particles = gprtBufferGetHandle(particleBuffer);

  // also assign particles to raygen
  raygenData.particles = gprtBufferGetHandle(particleBuffer);

  // same parameters go to these compute programs too
  ParticleData *genParticlesData = gprtComputeGetParameters(GenParticles);
  ParticleData *genRBFBoundsData = gprtComputeGetParameters(GenRBFBounds);
  *genParticlesData = *particleRecord;
  *genRBFBoundsData = *particleRecord;
  *splatRayGenData = *rbfRayGenData = *voxelRayGenData = raygenData;

  // Upload parameters
  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

  // Upload some particles
  gprtBufferMap(particleBuffer);
  float4* particlePositions = gprtBufferGetPointer(particleBuffer);
  memcpy(particlePositions, particles.data(), sizeof(float4) * particles.size());
  gprtBufferUnmap(particleBuffer);

  // Generate bounding boxes for those particles
  gprtComputeLaunch1D(context, GenRBFBounds, particles.size() / particlesPerLeaf);

  // Now we can build the tree
  GPRTAccel particleAccel = gprtAABBAccelCreate(context, 1, &particleGeom);
  gprtAccelBuild(context, particleAccel);
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, &particleAccel);
  gprtAccelBuild(context, world);

  // Assign tree to raygen parameters
  raygenData.world = gprtAccelGetHandle(world);

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
  auto minMaxVolume = gprtDeviceBufferCreate<float2>(
      context, ddaGridResolution * ddaGridResolution * ddaGridResolution, nullptr);
  auto majorantVolume = gprtDeviceBufferCreate<float>(
      context, ddaGridResolution * ddaGridResolution * ddaGridResolution, nullptr);
  raygenData.minMaxVolume = gprtBufferGetHandle(minMaxVolume);  
  raygenData.majorants = gprtBufferGetHandle(majorantVolume);
  uint3 ddaDimensions = uint3(ddaGridResolution, ddaGridResolution, ddaGridResolution);
  raygenData.ddaDimensions = ddaDimensions;

  // copy raygen params
  *splatRayGenData = *rbfRayGenData = *voxelRayGenData = raygenData;
  gprtBuildShaderBindingTable(context);

  // compute minmax ranges
  {
    // auto params1 = gprtComputeGetParameters<RayGenData>(ClearMinMaxGrid);
    auto params = gprtComputeGetParameters<RayGenData>(MinMaxRBFBounds);
    *params = raygenData;
    gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

    uint64_t numVoxels = ddaGridResolution * ddaGridResolution * ddaGridResolution;
    // gprtComputeLaunch1D(context, ClearMinMaxGrid, numVoxels);
    gprtBufferClear(minMaxVolume);    
    gprtComputeLaunch1D(context, MinMaxRBFBounds, particles.size());
  }

  ImGG::GradientWidget gradient_widget{};

  bool majorantsOutOfDate = true;
  bool voxelized = false;
  bool firstFrame = true;
  double xpos = 0.f, ypos = 0.f;
  double lastxpos, lastypos;
  uint32_t frameID = 1;
  do {
    ImGuiIO &io = ImGui::GetIO();
    ImGui::NewFrame();

    static int mode = 0;
    if (ImGui::RadioButton("Splatting", &mode, 0))
      frameID = 1;
    if (ImGui::RadioButton("RBF Query", &mode, 1))
      frameID = 1;
    if (ImGui::RadioButton("Voxelized", &mode, 2))
      frameID = 1;

    if (gradient_widget.widget("My Gradient") || firstFrame) {
      auto make_8bit = [](const float f) -> uint32_t {
        return std::min(255, std::max(0, int(f * 256.f)));
      };

      auto make_rgba = [make_8bit](float4 color) -> uint32_t {
        float gamma = 2.2;
        color =
            pow(color, float4(1.0f / gamma, 1.0f / gamma, 1.0f / gamma, 1.0f));
        return (make_8bit(color.x) << 0) + (make_8bit(color.y) << 8) +
               (make_8bit(color.z) << 16) + (make_8bit(color.w) << 24);
      };

      gprtTextureMap(colormap);
      uint32_t *ptr = gprtTextureGetPointer(colormap);
      for (uint32_t i = 0; i < 256; ++i) {
        auto result = gradient_widget.gradient().at(
            ImGG::RelativePosition(float(i + 1) / 257.f));
        ptr[i] = make_rgba(float4(result.x, result.y, result.z, result.w));
      }
      gprtTextureUnmap(colormap);

      frameID = 1;

      voxelized = false;
      majorantsOutOfDate = true;
    }

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
    int ctrl_state = gprtGetKey(context, GPRT_KEY_LEFT_CONTROL);

    // close window on Ctrl-W press
    if (w_state && ctrl_state) {
      break;
    }
    // close window on Ctrl-C press
    if (c_state && ctrl_state) {
      break;
    }

    // If we click the mouse, we should rotate the camera
    if (state == GPRT_PRESS && !io.WantCaptureMouse || firstFrame) {
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

      frameID = 1;
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

      frameID = 1;
    }

    if (mstate == GPRT_PRESS && !io.WantCaptureMouse) {
      float4 position = {lookFrom.x, lookFrom.y, lookFrom.z, 1.f};
      float4 pivot = {lookAt.x, lookAt.y, lookAt.z, 1.0};
      float3 lookRight = cross(lookUp, normalize(pivot - position).xyz());

      float3 translation = lookRight * dx + lookUp * -dy;
      translation = translation * .01f;

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

      frameID = 1;
    }

    static float clampMaxCumulativeValue = 1.f;
    static float unit = 0.1f;
    if (ImGui::SliderFloat("clamp max cumulative value",
                           &clampMaxCumulativeValue, 0.f, 10.f)) {
      frameID = 1;
      majorantsOutOfDate = true;
      voxelized = false;
    }
    if (ImGui::InputFloat("delta tracking unit value", &unit))
      frameID = 1;

    unit = std::max(unit, .001f);
    static float azimuth = 0.f;
    static float elevation = 0.f;
    static float ambient = .5f;

    if (ImGui::SliderFloat("azimuth", &azimuth, 0.f, 1.f))
      frameID = 1;
    if (ImGui::SliderFloat("elevation", &elevation, -1.f, 1.f))
      frameID = 1;
    if (ImGui::SliderFloat("ambient", &ambient, 0.f, 1.f))
      frameID = 1;
    ImGui::EndFrame();

    raygenData.frameID = frameID;
    raygenData.clampMaxCumulativeValue = clampMaxCumulativeValue;
    raygenData.unit = unit;
    raygenData.light.ambient = ambient;
    raygenData.light.elevation = elevation;
    raygenData.light.azimuth = azimuth;

    particleRecord->clampMaxCumulativeValue = clampMaxCumulativeValue;

    // copy raygen params
    *splatRayGenData = *rbfRayGenData = *voxelRayGenData = raygenData;
    gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

    // if we need to, revoxelize
    if (mode == 2 && !voxelized) {
      auto accumParams =
          gprtComputeGetParameters<RayGenData>(AccumulateRBFBounds);
      auto avgParams = gprtComputeGetParameters<RayGenData>(AverageRBFBounds);
      *accumParams = *avgParams = raygenData;
      gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
      gprtBufferClear(voxelVolume);
      gprtBufferClear(voxelVolumeCount);
      gprtComputeLaunch1D(context, AccumulateRBFBounds, particles.size());
      gprtComputeLaunch1D(context, AverageRBFBounds,
                          dims.x * dims.y * dims.z);
      voxelized = true;
    }

    // if we need to, recompute majorants 
    if (majorantsOutOfDate) {

      gprtBufferClear(majorantVolume);

      auto params = gprtComputeGetParameters<RayGenData>(ComputeMajorantGrid);
      *params = raygenData;
      gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
      uint64_t numVoxels = ddaGridResolution * ddaGridResolution * ddaGridResolution;
      gprtComputeLaunch1D(context, ComputeMajorantGrid, numVoxels);
      majorantsOutOfDate = false;
    }

    gprtTextureClear(guiDepthAttachment);
    gprtTextureClear(guiColorAttachment);
    gprtGuiRasterize(context);

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

    gprtBufferPresent(context, frameBuffer);

    frameID++;
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
