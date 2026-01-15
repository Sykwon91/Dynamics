cmake_minimum_required(VERSION 3.13)

project(transformer
    VERSION 1.0
    LANGUAGES CXX
)
include_directories(include)

# 실행 파일
#add_executable(transformer
#    src/main.cpp
#)

add_library(transformer
    src/transformer.cpp
)


add_executable(transformertest src/main.cpp)
target_link_libraries(transformer PRIVATE ${CMAKE_SOURCE_DIR}/build/libtransformer.a)