cmake_minimum_required(VERSION 3.16)

project(3DMath VERSION 1.0.0)

add_library(3DMath 
    frustum.h frustum.cpp
    transform.h transform.cpp
    TransformationMatrices.h TransformationMatrices.cpp
)

target_include_directories(3DMath PUBLIC ./)
target_link_libraries(3DMath PUBLIC eigen)