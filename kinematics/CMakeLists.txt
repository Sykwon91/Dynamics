cmake_minimum_required(VERSION 3.10)
project(kinematics)



# 라이브러리 경로
include_directories(include)


add_library(kinematics
    src/kinematics.cpp
)
target_link_libraries(kinematics PRIVATE ${CMAKE_SOURCE_DIR}/lib/liblinearalgebra.a)


add_executable(kinematicstest src/main.cpp)
target_link_libraries(kinematicstest PRIVATE
    ${CMAKE_SOURCE_DIR}/build/libkinematics.a
    kinematics
)



target_compile_features(kinematics PRIVATE cxx_std_17)
