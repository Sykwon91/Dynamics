cmake_minimum_required(VERSION 3.10)
project(vehicleDynamics)

add_executable(vehicleDynamics src/main.cpp)

# 라이브러리 경로
include_directories(include)
target_link_libraries(vehicleDynamics PRIVATE ${CMAKE_SOURCE_DIR}/lib/libcollision.a)
target_link_libraries(vehicleDynamics PRIVATE ${CMAKE_SOURCE_DIR}/lib/libkinematics.a)
target_link_libraries(vehicleDynamics PRIVATE ${CMAKE_SOURCE_DIR}/lib/liblinearalgebra.a)

target_compile_features(vehicleDynamics PRIVATE cxx_std_17)
