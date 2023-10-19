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

// public "General Purpose Raytracing Toolkit" API
#include <gprt.h>

// An example particle importer
#include "importParticles.h"

// stb for image loading and storing
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

// Our shared data structures between host and device
#include "sharedCode.h"

// For parallel sorting of points along a hilbert curve
#include "hilbert.h"

/* imgui for a small user interface */
#include "imgui.h"
#include <imgui_gradient/imgui_gradient.hpp>

//argument parsing
#include <argparse/argparse.hpp>

// misc
#include <fstream>
#include <iostream>
#include <algorithm>
#include <execution>

extern GPRTProgram deviceCodeCommon;
extern GPRTProgram deviceCodeBounds;
extern GPRTProgram deviceCodeSplat;
extern GPRTProgram deviceCodeRBF;

// initial image resolution
const int2 fbSize = {1024, 1024};

uint32_t particlesPerLeaf = 1;
std::vector<std::vector<float4>> particles;
size_t maxNumParticles;

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

int main(int argc, char *argv[]) {
  argparse::ArgumentParser program("Attribute-Aware Radial Basis Functions");

  program.add_argument("--particles")
      .help("A path to our custom particles dataset (ending in .particles)")
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

  std::vector<std::vector<std::pair<uint64_t, float4>>> particleData;
  std::string particlesPath = program.get<std::string>("--particles");
  bool synthetic = false;
  if (particlesPath != "") {
    std::cout << "loading " << particlesPath << std::endl;
    std::vector<std::vector<float4>> particles = importParticles(particlesPath);
    // wrangle particles into a pair data structure for later sorting
    particleData.resize(particles.size());
    for (uint32_t i = 0; i < particles.size(); ++i) {
      particleData[i].resize(particles[i].size());
      for (uint32_t j = 0; j < particles[i].size(); ++j) {
        particleData[i][j].second = particles[i][j];
      }
    }
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

  uint32_t particlesPerLeafArg = program.get<uint32_t>("--particles-per-leaf");
  if (particlesPerLeafArg > 0) particlesPerLeaf = particlesPerLeafArg;
  std::cout << "Particles per leaf " << particlesPerLeaf << std::endl;

  size_t totalParticles = 0;
  for (size_t j = 0; j < particleData.size(); ++j) {
    totalParticles += particleData[j].size();
  }
  std::cout << "Total particles " << totalParticles << std::endl;
  std::cout << "Avg Particles Per Step "
            << totalParticles / float(particleData.size()) << std::endl;
  std::cout << "Num steps " << particleData.size() << std::endl;

  float3 aabb[2] = {{1e38f, 1e38f, 1e38f}, {-1e38f, -1e38f, -1e38f}};


  std::cout << "Computing space-time bounding box..." << std::endl;
  for (size_t j = 0; j < particleData.size(); ++j) {
    for (size_t i = 0; i < particleData[j].size(); ++i) {
      aabb[0] = linalg::min(aabb[0], particleData[j][i].second.xyz());
      aabb[1] = linalg::max(aabb[1], particleData[j][i].second.xyz());
    }
  }
  std::cout << " - Done!" << std::endl;

  // Now, we compute hilbert codes per-point
  std::cout << "Computing hilbert codes..." << std::endl;
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
  std::cout << " - Done!" << std::endl;

  std::cout << "Sorting points along hilbert curve..." << std::endl;
  for (size_t j = 0; j < particleData.size(); ++j) {
#ifdef _WIN32
    // not sure why this isn't working on windows currently.
    std::sort(particleData[j].begin(), particleData[j].end());
#else
    std::sort(std::execution::par_unseq, particleData[j].begin(),
              particleData[j].end());
#endif
  }
  std::cout << " - Done!" << std::endl;

  // here just transferring to a vector we can actually use.
  maxNumParticles = 0;

  float minScalarValue = +1e20f;
  float maxScalarValue = -1e20f;

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
  
  // normalize attributes
  if (maxScalarValue > minScalarValue) {
    for (size_t j = 0; j < particles.size(); ++j) {
      for (size_t i = 0; i < particles[j].size(); ++i) {
        particles[j][i].w = (particles[j][i].w - minScalarValue) /
                            (maxScalarValue - minScalarValue);
      }
    }
  }

  // Initialize some camera parameters
  float3 lookFrom, lookAt, lookUp;
  float cosFovy;
  std::vector<float> camParams = program.get<std::vector<float>>("--camera");
  if (camParams.size() > 0) {
    lookFrom = float3(camParams[0], camParams[1], camParams[2]);
    lookAt = float3(camParams[3], camParams[4], camParams[5]);
    lookUp = float3(camParams[6], camParams[7], camParams[8]);
    cosFovy = camParams[9];
  } else {
    // set focus to aabb
    lookUp = {0.f, -1.f, 0.f};
    lookAt = (aabb[1] + aabb[0]) * .5f;
    lookFrom = aabb[1];
    if (synthetic) {
      lookAt = {0.f, 0.f, 0.f};
      lookFrom = {-3.f, 3.f, 3.f};
    }
    cosFovy = 0.66f;
  }

  /// Setup GPRT, RT kernels, etc
  gprtRequestWindow(fbSize.x, fbSize.y, "RT Point Clouds");

  // We have two ray types, one for our attribute-aware radial basis functions, 
  // and one for a reference particle splatter
  gprtRequestRayTypeCount(2);

  int32_t GPU = 0; // for now, just pick the first available GPU
  GPRTContext context = gprtContextCreate(&GPU);
  GPRTModule moduleCommon = gprtModuleCreate(context, deviceCodeCommon);
  GPRTModule moduleBounds = gprtModuleCreate(context, deviceCodeBounds);
  GPRTModule moduleSplat = gprtModuleCreate(context, deviceCodeSplat);
  GPRTModule moduleRBF = gprtModuleCreate(context, deviceCodeRBF);

  // A kernel for computing particle bounding boxes
  auto GenRBFBounds =
      gprtComputeCreate<UnusedRecord>(context, moduleBounds, "GenRBFBounds");
  
  // A kernel for compositing imgui and handling temporal antialiasing 
  auto CompositeGui =
      gprtComputeCreate<RayGenData>(context, moduleCommon, "CompositeGui");

  // Custom intersection and anyhit programs for our particles.
  auto particleType = gprtGeomTypeCreate<UnusedRecord>(context, GPRT_AABBS);
  gprtGeomTypeSetIntersectionProg(particleType, 0, moduleRBF, "ParticleRBFIntersection");
  gprtGeomTypeSetAnyHitProg(particleType, 0, moduleRBF, "ParticleRBFAnyHit");
  gprtGeomTypeSetIntersectionProg(particleType, 1, moduleSplat,"ParticleSplatIntersection");
  gprtGeomTypeSetAnyHitProg(particleType, 1, moduleSplat, "ParticleSplatAnyHit");

  // Required (but currently unused) miss program
  GPRTMissOf<UnusedRecord> miss =
      gprtMissCreate<UnusedRecord>(context, moduleCommon, "miss");

  // Ray generation programs
  GPRTRayGenOf<RayGenData> ParticleSplatRayGen =
      gprtRayGenCreate<RayGenData>(context, moduleSplat, "ParticleSplatRayGen");

  GPRTRayGenOf<RayGenData> ParticleRBFRayGen =
      gprtRayGenCreate<RayGenData>(context, moduleRBF, "ParticleRBFRayGen");

  // Some buffers to store intermediate images
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

  // Attachments for our imgui rasterizer
  auto guiColorAttachment = gprtDeviceTextureCreate<uint32_t>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_SRGB, fbSize.x,
      fbSize.y, 1, false, nullptr);
  auto guiDepthAttachment = gprtDeviceTextureCreate<float>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_D32_SFLOAT, fbSize.x, fbSize.y,
      1, false, nullptr);
  gprtGuiSetRasterAttachments(context, guiColorAttachment, guiDepthAttachment);

  // Spatio-Temporal Blue Noise mask
  bool STBNFound = true;
  std::string path = STBN_DIR "stbn.png";
  std::string altpath = "./stbn.png";
  int texWidth, texHeight, texChannels;
  stbi_uc *pixels = stbi_load(path.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);
  if (!pixels) pixels = stbi_load(altpath.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);
  if (!pixels) STBNFound = false; 
  
  GPRTTextureOf<stbi_uc> stbnTexture;
  if (STBNFound) {
    stbnTexture = gprtDeviceTextureCreate<stbi_uc>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_UNORM, texWidth, texHeight, 1, false, pixels
    );
  } else {
    std::cout<<"WARNING: Could not find stbn.png in the current working directory! Force disabling blue noise..." << std::endl;
  }

  // Color map / radius map / density map for visualization
  auto colormap = gprtDeviceTextureCreate<uint8_t>(context, GPRT_IMAGE_TYPE_1D,
                                                   GPRT_FORMAT_R8G8B8A8_SRGB,
                                                   64, 1, 1, false, nullptr);

  auto radiusmap = gprtDeviceTextureCreate<uint8_t>(context, GPRT_IMAGE_TYPE_1D,
                                                    GPRT_FORMAT_R8G8B8A8_SRGB,
                                                    64, 1, 1, false, nullptr);

  auto densitymap = gprtDeviceTextureCreate<uint8_t>(
      context, GPRT_IMAGE_TYPE_1D, GPRT_FORMAT_R8G8B8A8_SRGB, 64, 1, 1, false,
      nullptr);



  // Some constant data made available for raygens
  RayGenData raygenData = {};
  raygenData.fbSize = fbSize;
  raygenData.imageBuffer = gprtBufferGetHandle(imageBuffer);
  raygenData.accumBuffer = gprtBufferGetHandle(accumBuffer);
  if (STBNFound) raygenData.stbnTexture = gprtTextureGetHandle(stbnTexture);
  raygenData.globalAABBMin = aabb[0];
  raygenData.globalAABBMax = aabb[1];
  gprtRayGenSetParameters(ParticleSplatRayGen, &raygenData);
  gprtRayGenSetParameters(ParticleRBFRayGen, &raygenData);

  // GPU buffers to store particle positions, attributes, and bounds
  auto particleBuffer =
      gprtDeviceBufferCreate<float4>(context, maxNumParticles, nullptr);
  int numAABBs = ((maxNumParticles + particlesPerLeaf - 1) / particlesPerLeaf);
  auto aabbBuffer =
      gprtDeviceBufferCreate<float3>(context, 2 * numAABBs, nullptr);

  // For this sample, we'll just have one particle geometry. 
  // This associates the geometry data (AABB bounds) with shader kernels
  auto particleGeom = gprtGeomCreate<UnusedRecord>(context, particleType);
  gprtAABBsSetPositions(particleGeom, aabbBuffer,
                        ((maxNumParticles + particlesPerLeaf - 1) /
                         particlesPerLeaf) /* just one aabb */);

  PushConstants rtConstants;
  rtConstants.colormap = gprtTextureGetHandle(colormap);
  rtConstants.radiusmap = gprtTextureGetHandle(radiusmap);
  rtConstants.densitymap = gprtTextureGetHandle(densitymap);
  rtConstants.particlesPerLeaf = particlesPerLeaf;
  rtConstants.particles = gprtBufferGetHandle(particleBuffer);
  
  TAAConstants taaConstants;
  taaConstants.disableTAA = false;
  taaConstants.fbSize = fbSize;
  taaConstants.frameBuffer = gprtBufferGetHandle(frameBuffer);
  taaConstants.guiTexture = gprtTextureGetHandle(guiColorAttachment);
  taaConstants.imageTexture = gprtTextureGetHandle(imageTexture);
  taaConstants.taaBuffer = gprtBufferGetHandle(taaBuffer);
  taaConstants.taaPrevBuffer = gprtBufferGetHandle(taaPrevBuffer);

  // Build the shader binding table associating records with kernels, 
  // geometry IDs, ray types, etc. Also compiles the kernels into a 
  // ray tracing pipeline
  gprtBuildShaderBindingTable(context);
  
  std::vector<std::pair<float, float4>> cmMarks;
  std::vector<std::pair<float, float4>> rmMarks;
  std::vector<std::pair<float, float4>> dmMarks;

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

  bool firstFrame = true;
  double xpos = 0.f, ypos = 0.f;
  double lastxpos, lastypos;
  rtConstants.accumID = 1;
  rtConstants.frameID = 1;

  GPRTAccel particleAccel = gprtAABBAccelCreate(context, 1, &particleGeom);
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, &particleAccel);

  float diagonal = length(aabb[1] - aabb[0]);

  int previousParticleFrame = -1;
  float previousParticleRadius = ((synthetic) ? 0.05f : .01f) * diagonal;
  float radiusArg = program.get<float>("--radius");
  bool playAnimation = true;
  rtConstants.disableBlueNoise = !STBNFound;
  std::stringstream frameStats;

  rtConstants.rbfRadius = previousParticleRadius;
  rtConstants.unit = previousParticleRadius * .1f;
  rtConstants.visualizeAttributes = true;
  rtConstants.light.azimuth = 0.f;
  rtConstants.light.elevation = 0.f;
  rtConstants.light.ambient = .5f;
  rtConstants.frameID = 0;

  do {
    ImGuiIO &io = ImGui::GetIO();
    ImGui::NewFrame();

    // Time controls
    static int particleFrame = 0;
    ImGui::SliderInt("Frame", &particleFrame, 0, particles.size() - 1);
    if (ImGui::Button("Play Animation")) playAnimation = true;
    if (ImGui::Button("Pause Animation")) playAnimation = false;
    if (playAnimation) {
      particleFrame++;
      if (particleFrame >= particles.size())
        particleFrame = 1;
      rtConstants.accumID = 1;

      if (synthetic) {
        rtConstants.light.azimuth = sin(gprtGetTime(context)) * .5 + .5;
        rtConstants.light.elevation = cos(gprtGetTime(context));
        rtConstants.accumID = 1;
      }
    }

    // Radius and colormap controls
    bool radiusEdited = ImGui::DragFloat("Particle Radius", &rtConstants.rbfRadius, 
                      0.0001f * diagonal, .0001f * diagonal, 1.f * diagonal, "%.5f");
    if (rtConstants.rbfRadius != previousParticleRadius && radiusArg > 0.f)
      rtConstants.rbfRadius = radiusArg;
    bool densityEdited = false;
          
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

      rtConstants.accumID = 1;
      gprtTextureUnmap(colormap);
    }

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

      rtConstants.accumID = 1;
      gprtTextureUnmap(radiusmap);
    }

    if (densitymapWidget.widget("RBF Density", grayscaleWidgetSettings) ||
        firstFrame) {
      densityEdited = true;
      gprtTextureMap(densitymap);
      uint8_t *ptr = gprtTextureGetPointer(densitymap);
      for (uint32_t i = 0; i < 64; ++i) {
        auto result =
            densitymapWidget.gradient().at(ImGG::RelativePosition(i / 63.f));
        ptr[i * 4 + 0] = make_8bit(result.x);
      }

      rtConstants.accumID = 1;
      gprtTextureUnmap(densitymap);
    }

    // Blue noise controls
    if (!STBNFound) {
      ImGui::BeginDisabled();
      ImGui::BeginTooltip();
      ImGui::TextUnformatted("WARNING: Spatio Temporal Blue Noise  \"stbn.png\" texture \n not found in the current directory");
      ImGui::EndTooltip();
    }
    if (ImGui::Checkbox("Disable Blue Noise", (bool*)&rtConstants.disableBlueNoise)) rtConstants.accumID = 1;
    if (!STBNFound) ImGui::EndDisabled();

    if (ImGui::Checkbox("Disable Temporal Antialiasing", (bool*)&taaConstants.disableTAA))
      rtConstants.accumID = 1;

    static int mode = 1;
    if (ImGui::RadioButton("AA-RBF (Ours)", &mode, 1))
      rtConstants.accumID = 1;
    if (ImGui::RadioButton("Splatting (Knoll 2019)", &mode, 0))
      rtConstants.accumID = 1;

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

    // close window on Ctrl-W press or Ctrl-C press
    if (w_state && ctrl_state || c_state && ctrl_state) break;
    
    // Flip the "up" direction
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
        z_state || firstFrame) {
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
      rtConstants.camera.pos = camera_pos;
      rtConstants.camera.dir_00 = camera_d00;
      rtConstants.camera.dir_du = camera_ddu;
      rtConstants.camera.dir_dv = camera_ddv;
      rtConstants.accumID = 1;
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
      rtConstants.camera.pos = lookFrom;
      rtConstants.accumID = 1;
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
      rtConstants.camera.pos = camera_pos;
      rtConstants.camera.dir_00 = camera_d00;
      rtConstants.camera.dir_du = camera_ddu;
      rtConstants.camera.dir_dv = camera_ddv;
      rtConstants.accumID = 1;
    }

    if (ImGui::InputFloat("step size", &rtConstants.unit, 0.0f, 0.0f, "%.4f"))
      rtConstants.accumID = 1;
    
    if (ImGui::Checkbox("Visualize Attributes", (bool*)&rtConstants.visualizeAttributes)) {
      rtConstants.accumID = 1;
    }

    rtConstants.unit = std::max(rtConstants.unit, .0001f);


    if (ImGui::SliderFloat("azimuth", &rtConstants.light.azimuth, 0.f, 1.f))
      rtConstants.accumID = 1;
    if (ImGui::SliderFloat("elevation", &rtConstants.light.elevation, -1.f, 1.f))
      rtConstants.accumID = 1;
    if (ImGui::SliderFloat("ambient", &rtConstants.light.ambient, 0.f, 1.f))
      rtConstants.accumID = 1;
    ImGui::EndFrame();

    bool frameChanged = previousParticleFrame != particleFrame;
    bool radiusChanged = previousParticleRadius != rtConstants.rbfRadius || radiusEdited || densityEdited;
    
    // Particle manipulation
    if ( frameChanged || radiusChanged) 
    {
      if (frameChanged) {
        // Upload some particles
        gprtBufferMap(particleBuffer);
        float4 *particlePositions = gprtBufferGetPointer(particleBuffer);
        memcpy(particlePositions, particles[particleFrame].data(),
              sizeof(float4) * particles[particleFrame].size());
        gprtBufferUnmap(particleBuffer);
      }

      rtConstants.numParticles = particles[particleFrame].size();
     
      // Generate bounding boxes for those particles
      // note, unneeded boxes will be inactivated.
      BoundsConstants bc;
      bc.aabbs = gprtBufferGetHandle(aabbBuffer);
      bc.numAABBs = numAABBs;
      bc.particles = gprtBufferGetHandle(particleBuffer);
      bc.numParticles = particles[particleFrame].size();
      bc.particlesPerLeaf = rtConstants.particlesPerLeaf;
      bc.rbfRadius = rtConstants.rbfRadius;
      bc.radiusmap = gprtTextureGetHandle(radiusmap);
      int numWorkGroups = (numAABBs + 1023) / 1024; // 1024 threads per workgroup
      gprtComputeLaunch1D(context, GenRBFBounds, numWorkGroups, bc);

      // Build an entirely new tree
      if (radiusChanged) {
        gprtAccelBuild(context, particleAccel, GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE);
        gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);
        rtConstants.world = gprtAccelGetHandle(world);

        // account for newly introduced geometry to the SBT
        gprtBuildShaderBindingTable(context, GPRT_SBT_GEOM);
      }
      // Refit the tree if particles didn't move
      else {
        gprtAccelUpdate(context, particleAccel);
        gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);
      }

      rtConstants.accumID = 1;
      previousParticleFrame = particleFrame;
      previousParticleRadius = rtConstants.rbfRadius;
    }

    // Render the user interface
    gprtTextureClear(guiDepthAttachment);
    gprtTextureClear(guiColorAttachment);
    gprtGuiRasterize(context);
    
    gprtBeginProfile(context);

    switch (mode) {
    case 0:
      gprtRayGenLaunch2D(context, ParticleSplatRayGen, fbSize.x, fbSize.y, rtConstants);
      break;
    case 1:
      gprtRayGenLaunch2D(context, ParticleRBFRayGen, fbSize.x, fbSize.y, rtConstants);
      break;
    default:
      break;
    }

    gprtBufferTextureCopy(context, imageBuffer, imageTexture, 0, 0, 0, 0, 0, 0,
                          fbSize.x, fbSize.y, 1);

    gprtComputeLaunch2D(context, CompositeGui, fbSize.x, fbSize.y, taaConstants);

    gprtBufferPresent(context, frameBuffer);

    rtConstants.accumID++;
    rtConstants.frameID++;

    gprtBufferCopy(context, taaBuffer, taaPrevBuffer, 0, 0,
                   fbSize.x * fbSize.y);

  } while (!gprtWindowShouldClose(context));

  gprtBufferDestroy(particleBuffer);
  gprtBufferDestroy(aabbBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(ParticleSplatRayGen);
  gprtRayGenDestroy(ParticleRBFRayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(particleAccel);
  gprtAccelDestroy(world);
  gprtGeomDestroy(particleGeom);
  gprtGeomTypeDestroy(particleType);
  gprtModuleDestroy(moduleCommon);
  gprtModuleDestroy(moduleBounds);
  gprtModuleDestroy(moduleSplat);
  gprtModuleDestroy(moduleRBF);
  gprtContextDestroy(context);
}
