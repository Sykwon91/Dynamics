cmake_minimum_required(VERSION 3.13)

project(missileDynamics VERSION 1.0 LANGUAGES CXX)

if(NOT TARGET linearalgebra)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../LinearAlgebra ${CMAKE_CURRENT_BINARY_DIR}/LinearAlgebra)
endif()

if(NOT TARGET kinematics)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Kinematics ${CMAKE_CURRENT_BINARY_DIR}/Kinematics)
endif()

if(NOT TARGET raylib)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../raylib ${CMAKE_CURRENT_BINARY_DIR}/raylib)
endif()

set(SRC_COMMON
    src/missile.cpp
)

add_library(missileDynamics ${SRC_COMMON})
target_include_directories(missileDynamics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(missileDynamics PUBLIC kinematics linearalgebra)
target_compile_features(missileDynamics PRIVATE cxx_std_17)

add_executable(missileDynamicstest src/main.cpp)
target_link_libraries(missileDynamicstest PRIVATE missileDynamics raylib)
