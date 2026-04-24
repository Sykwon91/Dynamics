cmake_minimum_required(VERSION 3.13)

project(collision_module VERSION 1.0 LANGUAGES CXX)

if(NOT TARGET linearalgebra)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../LinearAlgebra ${CMAKE_CURRENT_BINARY_DIR}/LinearAlgebra)
endif()

add_library(collision src/collision.cpp)

target_include_directories(collision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(collision PUBLIC linearalgebra)
target_compile_features(collision PUBLIC cxx_std_17)
set_target_properties(collision PROPERTIES OUTPUT_NAME collision)

add_executable(collsion src/main.cpp)
target_link_libraries(collsion PRIVATE collision)
