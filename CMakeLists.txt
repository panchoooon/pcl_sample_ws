cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_practice)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#for cloud_viewer.cpp
add_executable (cloud_viewer cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

#for pcl_read.cpp
add_executable (pcd_read pcd_read.cpp)
target_link_libraries (pcd_read ${PCL_LIBRARIES})

#for oppenni_viewer_simple.cpp
add_executable (openni_viewer_simple openni_viewer_simple.cpp)
target_link_libraries (openni_viewer_simple ${PCL_LIBRARIES})

#for create_window.c
add_executable(create_window create_window.c)
find_package(X11 REQUIRED)
target_link_libraries(create_window ${X11_LIBRARIES})

#for passthrough.cpp
add_executable (passthrough passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES})

#for InOutPra.cpp
add_executable (InOutPra InOutPra.cpp)

#for statistical_removal.cpp
add_executable (statistical_removal statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

#for remove_outlier.cpp
add_executable (remove_outliers remove_outliers.cpp)
target_link_libraries (remove_outliers ${PCL_LIBRARIES})

#for voxel_grid.cpp
add_executable (voxel_grid voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})

#for kdtree_search.cpp
add_executable(kdtree_search kdtree_search.cpp)
target_link_libraries (kdtree_search ${PCL_LIBRARIES})

#for extract_indices.cpp
add_executable(extract_indices extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})

#for my_clustering.cpp
add_executable(my_clustering my_clustering.cpp)
target_link_libraries (my_clustering ${PCL_LIBRARIES})

#range_image_creation.cpp
add_executable(range_image_creation range_image_creation.cpp)
target_link_libraries (range_image_creation ${PCL_LIBRARIES})



