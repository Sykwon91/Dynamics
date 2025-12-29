cmake_minimum_required(VERSION 3.13)

project(point_cylinder_collision
    VERSION 1.0
    LANGUAGES CXX
)
include_directories(include)

# 실행 파일
#add_executable(point_cylinder_collision
#    src/main.cpp
#)

add_library(collision
    src/collision.cpp
)


add_executable(collsion src/main.cpp)
target_link_libraries(collsion PRIVATE ${CMAKE_SOURCE_DIR}/build/libcollision.a)