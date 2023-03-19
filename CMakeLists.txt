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

option(BUILD_MMPLD "Build MMPLD Library for University of Stuttgart Data" ON)
if (${BUILD_MMPLD})
add_library(mmpld INTERFACE)
target_include_directories(mmpld INTERFACE ${CMAKE_CURRENT_LIST_DIR}/3rdParty/mmpld_io/include/)
endif()

option(BUILD_GENERICIO "Build GENERICIO Library for HACC" ON)
if (${BUILD_GENERICIO})
add_subdirectory(3rdParty/genericio)
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

add_executable(kdTreeBuiler kdTreeBuilder.cpp memory.cpp)

if (${BUILD_ARBORX})
set(LIBRARIES ${LIBRARIES} ArborX::ArborX)
set(COMPILE_DEFINITIONS ${COMPILE_DEFINITIONS} -DARBORX)
endif()

if (${BUILD_MMPLD})
set(COMPILE_DEFINITIONS ${COMPILE_DEFINITIONS} -DMMPLD)
set(LIBRARIES ${LIBRARIES} mmpld)
endif()

if (${BUILD_GENERICIO})
set(COMPILE_DEFINITIONS ${COMPILE_DEFINITIONS} -DGENERICIO)
set(LIBRARIES ${LIBRARIES} genericio)
endif()

find_package(TBB)

if (${TBB_FOUND})
set(LIBRARIES ${LIBRARIES} TBB::tbb)
else(${TBB_FOUND})
message(WARN "Could NOT find TBB")
endif()

target_link_libraries(viewer PUBLIC ${LIBRARIES})
target_compile_definitions(viewer PUBLIC ${COMPILE_DEFINITIONS})

target_link_libraries(kdTreeBuiler PUBLIC ${LIBRARIES})
target_compile_definitions(kdTreeBuiler PUBLIC ${COMPILE_DEFINITIONS})