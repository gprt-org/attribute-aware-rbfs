cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(point-clouds CXX)

add_subdirectory(3rdParty/gprt)

add_subdirectory(3rdParty/imgui_gradient)
target_include_directories(imgui_gradient PRIVATE 3rdParty/gprt/3rdParty/)

embed_devicecode(
  OUTPUT_TARGET
    deviceCodeCommon
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCodeCommon.hlsl
)

embed_devicecode(
  OUTPUT_TARGET
    deviceCodeSplat
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCodeSplat.hlsl
)

embed_devicecode(
  OUTPUT_TARGET
    deviceCodeRBF
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCodeRBF.hlsl
)

embed_devicecode(
  OUTPUT_TARGET
    deviceCodeRaster
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCodeRaster.hlsl
)

add_executable(viewer hostCode.cpp)
target_link_libraries(viewer PUBLIC deviceCodeCommon deviceCodeSplat deviceCodeRBF deviceCodeRaster gprt::gprt imgui_gradient::imgui_gradient)
