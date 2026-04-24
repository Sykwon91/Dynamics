cmake_minimum_required(VERSION 3.13)

project(terrainmechanics VERSION 1.0 LANGUAGES CXX)



if(NOT TARGET linearalgebra)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../LinearAlgebra ${CMAKE_CURRENT_BINARY_DIR}/LinearAlgebra)
endif()

add_library(terrainmechanics src/TerrainMechanics.cpp)

target_include_directories(terrainmechanics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(terrainmechanics PUBLIC linearalgebra)
target_compile_features(terrainmechanics PUBLIC cxx_std_17)
set_target_properties(terrainmechanics PROPERTIES OUTPUT_NAME terrainmechanics)

add_executable(terrainmechanicstest src/main.cpp)
target_link_libraries(terrainmechanicstest PRIVATE terrainmechanics)
