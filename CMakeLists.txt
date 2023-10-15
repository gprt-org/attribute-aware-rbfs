cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(point-clouds CXX)

add_subdirectory(3rdParty/argparse)
add_subdirectory(3rdParty/GPRT)

add_subdirectory(3rdParty/imgui_gradient)
target_include_directories(imgui_gradient PUBLIC 3rdParty/GPRT/3rdParty/)

embed_devicecode(
  OUTPUT_TARGET
    deviceCodeCommon
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCodeCommon.hlsl
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rng.h
)

embed_devicecode(
  OUTPUT_TARGET
    deviceCodeSplat
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCodeSplat.hlsl
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rng.h
)

embed_devicecode(
  OUTPUT_TARGET
    deviceCodeRBF
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCodeRBF.hlsl
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rng.h
)

set(LIBRARIES 
  ${LIBRARIES}
  deviceCodeCommon 
  deviceCodeSplat 
  deviceCodeRBF
  gprt::gprt 
  imgui_gradient::imgui_gradient 
  argparse 
)
set(COMPILE_DEFINITIONS)

set(SOURCES
  hostCode.cpp
  IniFile.cpp
)
add_executable(viewer)
target_sources(viewer PRIVATE ${SOURCES})

set(COMPILE_DEFINITIONS ${COMPILE_DEFINITIONS} -DSTBN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

find_package(TBB)
if (${TBB_FOUND})
set(LIBRARIES ${LIBRARIES} TBB::tbb)
else(${TBB_FOUND})
message(WARN " Could NOT find TBB")
endif()

target_link_libraries(viewer PUBLIC ${LIBRARIES})
target_compile_definitions(viewer PUBLIC ${COMPILE_DEFINITIONS})
