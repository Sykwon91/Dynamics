cmake_minimum_required(VERSION 3.13)

project(odesolver VERSION 1.0 LANGUAGES CXX)

if(NOT TARGET linearalgebra)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../LinearAlgebra ${CMAKE_CURRENT_BINARY_DIR}/LinearAlgebra)
endif()

if(NOT TARGET kinematics)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Kinematics ${CMAKE_CURRENT_BINARY_DIR}/Kinematics)
endif()

add_library(odesolver src/ODESolver.cpp)

target_include_directories(odesolver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(odesolver PUBLIC linearalgebra kinematics)
target_compile_features(odesolver PUBLIC cxx_std_17)
set_target_properties(odesolver PROPERTIES OUTPUT_NAME odesolver)

add_executable(odesolvertest src/main.cpp)
target_link_libraries(odesolvertest PRIVATE odesolver)
