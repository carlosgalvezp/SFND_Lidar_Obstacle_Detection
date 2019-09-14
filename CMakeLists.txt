cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_subdirectory(src/quiz/ransac)
add_subdirectory(src/quiz/cluster)

add_library(process_point_clouds src/processPointClouds.cpp)
target_link_libraries(process_point_clouds PUBLIC ransac ${PCL_LIBRARIES})

add_executable (environment src/environment.cpp src/render/render.cpp)
target_link_libraries (environment process_point_clouds)
