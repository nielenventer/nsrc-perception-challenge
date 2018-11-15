cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cloud_viewer)

find_package( PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Viewer
add_executable( cloud_viewer cloud_viewer.cpp )
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

# Transformer
add_executable( cloud_transformer cloud_transformer.cpp )
target_link_libraries (cloud_transformer ${PCL_LIBRARIES})

# Find 3D 
add_executable( cloud_find_xyz cloud_find_xyz.cpp )
target_link_libraries (cloud_find_xyz ${PCL_LIBRARIES})

# Find in cloud
add_executable( cloud_find_in_new_cloud cloud_find_in_new_cloud.cpp )
target_link_libraries (cloud_find_in_new_cloud ${PCL_LIBRARIES})

# Compare sensor clouds
add_executable( cloud_compare_sensors cloud_compare_sensors.cpp )
target_link_libraries (cloud_compare_sensors ${PCL_LIBRARIES})
