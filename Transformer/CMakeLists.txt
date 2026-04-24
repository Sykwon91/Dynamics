cmake_minimum_required(VERSION 3.13)

project(transformer VERSION 1.0 LANGUAGES CXX)

add_library(transformer src/transformer.cpp)

target_include_directories(transformer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(transformer PUBLIC cxx_std_17)
set_target_properties(transformer PROPERTIES OUTPUT_NAME transformer)

add_executable(transformertest src/main.cpp)
target_link_libraries(transformertest PRIVATE transformer)
