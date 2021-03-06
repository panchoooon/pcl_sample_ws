cmake_minimum_required(VERSION 2.8.3)
project(pcl_practice)
find_package(PCL 1.2 REQUIRED)
#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  rospy
#  std_msgs
#  sensor_msgs
#  pcl_ros
#  pcl_conversions
#  image_transport
#  cv_bridge
#  opencv_apps
#)

find_package(catkin REQUIRED COMPONENTS)
find_package(roscpp)
find_package(rospy)
find_package(std_msgs)
find_package(sensor_msgs)
find_package(pcl_ros)
find_package(pcl_conversions)
find_package(image_transport)
find_package(cv_bridge)
find_package(OpenCV REQUIRED)
#find_package(opencv_apps)
find_package(image_transport)



include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

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
target_link_libraries(my_clustering ${catkin_LIBRARIES})
target_link_libraries (my_clustering ${PCL_LIBRARIES})
target_link_libraries(my_clustering ${OpenCV_LIBRARIES})


#range_image_creation.cpp
add_executable(range_image_creation range_image_creation.cpp) 
target_link_libraries (range_image_creation ${PCL_LIBRARIES}) #必ずadd_exxecutableより後

#range_image_visualization.cpp
add_executable(range_image_visualization range_image_visualization.cpp) 
target_link_libraries (range_image_visualization ${PCL_LIBRARIES}) 

#for img_obj_proc.cpp  
#add_executable(img_obj_proc img_obj_proc.cpp)
#target_link_libraries (img_obj_proc ${PCL_LIBRARIES})

#for test_include.cpp
add_executable(test_include ./header/func.h test_include.cpp)

#for my_create_rangeImage.cpp
add_executable(my_create_rangeImage my_create_rangeImage.cpp) 
target_link_libraries (my_create_rangeImage ${PCL_LIBRARIES}) 
target_link_libraries(my_create_rangeImage ${OpenCV_LIBRARIES})

#for rosPcl.cpp
#add_executable(rosPcl rosPcl.cpp)
#target_link_libraries (rosPcl ${PCL_LIBRARIES}) 
#target_link_libraries(rosPcl ${catkin_LIBRARIES}) 

#for region_growing_rgb_segmentation.cpp
add_executable(region_growing_rgb_segmentation region_growing_rgb_segmentation.cpp) 
target_link_libraries (region_growing_rgb_segmentation ${PCL_LIBRARIES}) 

#colorbase_segmentation.cpp
add_executable(colorbase_segmentation colorbase_segmentation.cpp) 
target_link_libraries (colorbase_segmentation ${PCL_LIBRARIES}) 


#accessPoints.cpp
add_executable(accessPoints accessPoints.cpp) 
target_link_libraries (accessPoints ${PCL_LIBRARIES}) 

#pcltocv.cpp
add_executable(pcltocv pcltocv.cpp) 
target_link_libraries (pcltocv ${PCL_LIBRARIES}) 
target_link_libraries(pcltocv ${OpenCV_LIBRARIES})
