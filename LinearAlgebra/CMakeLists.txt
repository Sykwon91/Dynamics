cmake_minimum_required(VERSION 3.13)

project(linearalgebra
    VERSION 1.0
    LANGUAGES CXX
)
include_directories(include)

# 실행 파일
#add_executable(point_cylinder_collision
#    src/main.cpp
#)

add_library(linearalgebra
    src/linearalgebra.cpp
)


add_executable(linearalgebratest src/main.cpp)
target_link_libraries(linearalgebratest PRIVATE ${CMAKE_SOURCE_DIR}/build/liblinearalgebra.a)