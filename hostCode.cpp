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

#define LOG(message)                                                           \
  std::cout << GPRT_TERMINAL_BLUE;                                             \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                        \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                       \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram deviceCode;

// initial image resolution
const int2 fbSize = {1000, 1000};

// Initial camera parameters
float3 lookFrom = {3.5f, 3.5f, 3.5f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

uint32_t numParticles = 10000;
float rbfRadius = .05f;

#include <iostream>
int main(int ac, char **av) {
  gprtRequestWindow(fbSize.x, fbSize.y, "RT Point Clouds");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, deviceCode);

  auto GenParticles =
      gprtComputeCreate<ParticleData>(context, module, "GenParticles");
  auto GenRBFBounds =
      gprtComputeCreate<ParticleData>(context, module, "GenRBFBounds");

  auto particleSplatType =
      gprtGeomTypeCreate<ParticleData>(context, GPRT_AABBS);
  gprtGeomTypeSetIntersectionProg(particleSplatType, 0, module,
                                  "ParticleSplatIntersection");
  gprtGeomTypeSetAnyHitProg(particleSplatType, 0, module,
                            "ParticleSplatAnyHit");
  GPRTMissOf<MissProgData> miss =
      gprtMissCreate<MissProgData>(context, module, "miss");
  GPRTRayGenOf<RayGenData> ParticleSplatRayGen =
      gprtRayGenCreate<RayGenData>(context, module, "ParticleSplatRayGen");

  auto frameBuffer =
      gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);
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

  RayGenData *rayGenData = gprtRayGenGetParameters(ParticleSplatRayGen);
  rayGenData->frameBuffer = gprtBufferGetHandle(frameBuffer);
  rayGenData->colormap = gprtTextureGetHandle(colormap);
  rayGenData->colormapSampler = gprtSamplerGetHandle(sampler);
  rayGenData->guiTexture = gprtTextureGetHandle(guiColorAttachment);

  float3 initialAABB[2] = {
      {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
       std::numeric_limits<float>::max()},
      {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
       -std::numeric_limits<float>::max()},
  };
  auto globalAABBBuffer =
      gprtDeviceBufferCreate<float3>(context, 2, initialAABB);
  rayGenData->globalAABB = gprtBufferGetHandle(globalAABBBuffer);
  rayGenData->rbfRadius = rbfRadius;

  MissProgData *missData = gprtMissGetParameters(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  auto particleBuffer =
      gprtDeviceBufferCreate<float4>(context, numParticles, nullptr);
  auto aabbBuffer =
      gprtDeviceBufferCreate<float3>(context, 2 * numParticles, nullptr);
  auto particleGeom = gprtGeomCreate<ParticleData>(context, particleSplatType);
  gprtAABBsSetPositions(particleGeom, aabbBuffer,
                        numParticles /* just one aabb */);

  ParticleData *particleRecord = gprtGeomGetParameters(particleGeom);
  particleRecord->numParticles = numParticles;
  particleRecord->rbfRadius = rbfRadius;
  particleRecord->aabbs = gprtBufferGetHandle(aabbBuffer);
  particleRecord->globalAABB = gprtBufferGetHandle(globalAABBBuffer);
  particleRecord->particles = gprtBufferGetHandle(particleBuffer);

  // also assign particles to raygen
  rayGenData->particles = gprtBufferGetHandle(particleBuffer);

  // same parameters go to these compute programs too
  ParticleData *genParticlesData = gprtComputeGetParameters(GenParticles);
  ParticleData *genRBFBoundsData = gprtComputeGetParameters(GenRBFBounds);
  *genParticlesData = *particleRecord;
  *genRBFBoundsData = *particleRecord;

  // Upload parameters
  gprtBuildShaderBindingTable(context);

  // Generate some particles
  gprtComputeLaunch1D(context, GenParticles, numParticles);

  // Generate bounding boxes for those particles
  gprtComputeLaunch1D(context, GenRBFBounds, numParticles);

  gprtBufferMap(globalAABBBuffer);
  float3 *tmp = gprtBufferGetPointer(globalAABBBuffer);
  std::cout << tmp[0].x << " " << tmp[0].y << " " << tmp[0].z << std::endl;
  std::cout << tmp[1].x << " " << tmp[1].y << " " << tmp[1].z << std::endl;
  gprtBufferUnmap(globalAABBBuffer);

  // Now we can build the tree
  GPRTAccel particleAccel = gprtAABBAccelCreate(context, 1, &particleGeom);
  gprtAccelBuild(context, particleAccel);
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, &particleAccel);
  gprtAccelBuild(context, world);

  // Assign tree to raygen parameters
  rayGenData->world = gprtAccelGetHandle(world);

  gprtBuildShaderBindingTable(context);

  ImGG::GradientWidget gradient_widget{};

  bool firstFrame = true;
  double xpos = 0.f, ypos = 0.f;
  double lastxpos, lastypos;
  do {
    ImGuiIO &io = ImGui::GetIO();
    ImGui::NewFrame();

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
    }

    float speed = .001f;
    lastxpos = xpos;
    lastypos = ypos;
    gprtGetCursorPos(context, &xpos, &ypos);
    if (firstFrame) {
      lastxpos = xpos;
      lastypos = ypos;
    }
    int state = gprtGetMouseButton(context, GPRT_MOUSE_BUTTON_LEFT);

    // If we click the mouse, we should rotate the camera
    // Here, we implement some simple camera controls
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
      float xAngle = (lastxpos - xpos) * deltaAngleX;
      float yAngle = (lastypos - ypos) * deltaAngleY;

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
      RayGenData *raygenData = gprtRayGenGetParameters(ParticleSplatRayGen);
      raygenData->camera.pos = camera_pos;
      raygenData->camera.dir_00 = camera_d00;
      raygenData->camera.dir_du = camera_ddu;
      raygenData->camera.dir_dv = camera_ddv;
    }

    ImGui::EndFrame();

    gprtTextureClear(guiDepthAttachment);
    gprtTextureClear(guiColorAttachment);
    gprtGuiRasterize(context);

    gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);

    gprtRayGenLaunch2D(context, ParticleSplatRayGen, fbSize.x, fbSize.y);
    gprtBufferPresent(context, frameBuffer);
  } while (!gprtWindowShouldClose(context));

  LOG("cleaning up ...");

  gprtBufferDestroy(particleBuffer);
  gprtBufferDestroy(aabbBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(ParticleSplatRayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(particleAccel);
  gprtAccelDestroy(world);
  gprtGeomDestroy(particleGeom);
  gprtGeomTypeDestroy(particleSplatType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
