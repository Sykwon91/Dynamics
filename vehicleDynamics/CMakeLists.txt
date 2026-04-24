cmake_minimum_required(VERSION 3.13)

project(vehicleDynamics VERSION 1.0 LANGUAGES CXX)

if(NOT TARGET linearalgebra)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../LinearAlgebra ${CMAKE_CURRENT_BINARY_DIR}/LinearAlgebra)
endif()

if(NOT TARGET kinematics)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../kinematics ${CMAKE_CURRENT_BINARY_DIR}/kinematics)
endif()

if(NOT TARGET collision)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Collsion ${CMAKE_CURRENT_BINARY_DIR}/Collsion)
endif()


if(NOT TARGET terrainmechanics)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../TerrainMechanics ${CMAKE_CURRENT_BINARY_DIR}/TerrainMechanics)
endif()

add_executable(vehicleDynamics src/main.cpp)

target_link_libraries(vehicleDynamics PRIVATE collision kinematics linearalgebra terrainmechanics)
target_compile_features(vehicleDynamics PRIVATE cxx_std_17)
