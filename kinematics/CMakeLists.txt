cmake_minimum_required(VERSION 3.10)
project(kinematics)

add_executable(kinematics src/main.cpp)

# 라이브러리 경로
include_directories(include)
target_link_libraries(kinematics PRIVATE ${CMAKE_SOURCE_DIR}/lib/libcollision.a)

target_compile_features(kinematics PRIVATE cxx_std_17)
