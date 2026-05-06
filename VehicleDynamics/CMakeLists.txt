cmake_minimum_required(VERSION 3.13)

project(vehicleDynamics VERSION 1.0 LANGUAGES CXX)

if(NOT TARGET linearalgebra)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../LinearAlgebra ${CMAKE_CURRENT_BINARY_DIR}/LinearAlgebra)
endif()

if(NOT TARGET kinematics)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Kinematics ${CMAKE_CURRENT_BINARY_DIR}/Kinematics)
endif()

if(NOT TARGET collision)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Collsion ${CMAKE_CURRENT_BINARY_DIR}/Collsion)
endif()

if(NOT TARGET terrainmechanics)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../TerrainMechanics ${CMAKE_CURRENT_BINARY_DIR}/TerrainMechanics)
endif()

if(NOT TARGET odesolver)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ODESolver ${CMAKE_CURRENT_BINARY_DIR}/ODESolver)
endif()

if(NOT TARGET visualization)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Visualization ${CMAKE_CURRENT_BINARY_DIR}/Visualization)
endif()


find_package(yaml-cpp REQUIRED)

add_library(vehicleDynamics src/vehicle.cpp)

target_include_directories(vehicleDynamics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(vehicleDynamics PUBLIC odesolver collision kinematics linearalgebra terrainmechanics visualization yaml-cpp)
target_compile_features(vehicleDynamics PRIVATE cxx_std_17)



add_executable(vehicleDynamicstest src/main.cpp)
target_include_directories(vehicleDynamicstest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(vehicleDynamicstest PRIVATE vehicleDynamics)
