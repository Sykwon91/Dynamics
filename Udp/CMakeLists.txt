cmake_minimum_required(VERSION 3.13)

project(udp VERSION 1.0 LANGUAGES CXX)

add_library(udp src/udp.cpp)
target_include_directories(udp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(udp PUBLIC cxx_std_17)

add_executable(udptest src/main.cpp)
target_link_libraries(udptest PRIVATE udp)
