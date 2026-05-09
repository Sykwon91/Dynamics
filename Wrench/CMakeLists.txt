cmake_minimum_required(VERSION 3.13)

project(wrench VERSION 1.0 LANGUAGES CXX)


if(NOT TARGET linearalgebra)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../LinearAlgebra ${CMAKE_CURRENT_BINARY_DIR}/LinearAlgebra)
endif()


add_library(wrench src/wrench.cpp)

target_include_directories(wrench PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(wrench PUBLIC linearalgebra)
target_compile_features(wrench PRIVATE cxx_std_17)