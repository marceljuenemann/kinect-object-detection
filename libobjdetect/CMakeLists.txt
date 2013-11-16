cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

# Point Cloud Library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Build this library
add_library(ObjectDetection core.cpp)
