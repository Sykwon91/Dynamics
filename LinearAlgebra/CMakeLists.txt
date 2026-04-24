cmake_minimum_required(VERSION 3.13)

project(linearalgebra VERSION 1.0 LANGUAGES CXX)

add_library(linearalgebra src/linearalgebra.cpp)

target_include_directories(linearalgebra PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(linearalgebra PUBLIC cxx_std_17)
set_target_properties(linearalgebra PROPERTIES OUTPUT_NAME linearalgebra)

add_executable(linearalgebratest src/main.cpp)
target_link_libraries(linearalgebratest PRIVATE linearalgebra)
