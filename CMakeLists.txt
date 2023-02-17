cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(point-clouds CXX)

add_subdirectory(3rdParty/gprt)

add_subdirectory(3rdParty/imgui_gradient)
target_include_directories(imgui_gradient PRIVATE 3rdParty/gprt/3rdParty/)

embed_devicecode(
  OUTPUT_TARGET
    deviceCode
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.hlsl
)

add_executable(viewer hostCode.cpp)
target_link_libraries(viewer PUBLIC deviceCode gprt::gprt imgui_gradient::imgui_gradient)
