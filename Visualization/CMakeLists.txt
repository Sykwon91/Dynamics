cmake_minimum_required(VERSION 3.13)

project(visualization VERSION 1.0 LANGUAGES CXX)

if(NOT TARGET linearalgebra)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../LinearAlgebra ${CMAKE_CURRENT_BINARY_DIR}/LinearAlgebra)
endif()

if(NOT TARGET collision)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Collsion ${CMAKE_CURRENT_BINARY_DIR}/Collsion)
endif()

add_library(visualization src/visualization.cpp)

target_include_directories(visualization PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(visualization PUBLIC collision linearalgebra)
target_compile_features(visualization PRIVATE cxx_std_17)
