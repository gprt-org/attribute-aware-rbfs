cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(point-clouds CXX)

add_subdirectory(3rdParty/gprt)

embed_devicecode(
  OUTPUT_TARGET
    deviceCode
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.hlsl
)

add_executable(viewer hostCode.cpp)
target_link_libraries(viewer PUBLIC deviceCode gprt::gprt)
