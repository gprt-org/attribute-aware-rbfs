cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(point-clouds CXX)

add_subdirectory(3rdParty/argparse)
add_subdirectory(3rdParty/GPRT)

add_subdirectory(3rdParty/imgui_gradient)
target_include_directories(imgui_gradient PUBLIC 3rdParty/GPRT/3rdParty/)

option(BUILD_ARBORX "Build ArborX Library for DBScan Dataset" ON)
if (${BUILD_ARBORX})
add_subdirectory(3rdParty/ArborX)
endif()

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
    deviceCodeVoxel
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCodeVoxel.hlsl
)

set(LIBRARIES 
  deviceCodeCommon 
  deviceCodeSplat 
  deviceCodeRBF
  deviceCodeVoxel 
  gprt::gprt 
  imgui_gradient::imgui_gradient 
  argparse 
)
set(COMPILE_DEFINITIONS)

add_executable(viewer hostCode.cpp)

if (${BUILD_ARBORX})
set(LIBRARIES ${LIBRARIES} ArborX::ArborX)
set(COMPILE_DEFINITIONS ${COMPILE_DEFINITIONS} -DARBORX)
endif()

find_package(TBB)

if (${TBB_FOUND})
set(LIBRARIES ${LIBRARIES} TBB::tbb)
else(${TBB_FOUND})
message(WARN "Could NOT find TBB")
endif()

target_link_libraries(viewer PUBLIC ${LIBRARIES})
target_compile_definitions(viewer PUBLIC ${COMPILE_DEFINITIONS})