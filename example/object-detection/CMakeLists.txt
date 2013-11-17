cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

# Output dir is bin (assumes build happens in build/ dir)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

# Project name
project(object-detection)

# Point Cloud Library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Object Detection Library
include_directories("../..")
add_subdirectory("../../libobjdetect" "${CMAKE_CURRENT_BINARY_DIR}/libobjdetect")

# Build executable and link
add_executable (object-detection object-detection.cpp)
target_link_libraries (object-detection ${PCL_LIBRARIES} ObjectDetection)
