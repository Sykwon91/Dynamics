cmake_minimum_required(VERSION 3.13)

project(kinematics VERSION 1.0 LANGUAGES CXX)

if(NOT TARGET linearalgebra)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../LinearAlgebra ${CMAKE_CURRENT_BINARY_DIR}/LinearAlgebra)
endif()

add_library(kinematics src/kinematics.cpp)

target_include_directories(kinematics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kinematics PUBLIC linearalgebra)
target_compile_features(kinematics PUBLIC cxx_std_17)
set_target_properties(kinematics PROPERTIES OUTPUT_NAME kinematics)

add_executable(kinematicstest src/main.cpp)
target_link_libraries(kinematicstest PRIVATE kinematics)
