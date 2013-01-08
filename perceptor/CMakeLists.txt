cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kinect_PCL_viewer)

add_definitions("-std=gnu++0x") 
find_package(PCL 1.2 REQUIRED)

message("${PCL_DEFINITIONS}\n")
message("${PCL_LIBRARIES}\n")
message("${PCL_LIBRARIES_DIRS}")
include_directories("/usr/include/pcl-1.6")
include_directories("/usr/include/eigen3") 
include_directories("/usr/include/openni") 
#link_directories("/home/wondereamer/local/lib") 
link_directories("/home/lib") 
#link_directories("/usr/local/lib")
add_definitions(${PCL_DEFINITIONS})

#set(ROS_BUILD_TYPE Release)
#set(PCL_LIBRARIES "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;/usr/lib/libboost_iostreams-mt.so;/usr/lib/libboost_mpi-mt.so;/usr/lib/libboost_serialization-mt.so;optimized;/usr/lib/libpcl_common.so;debug;/usr/lib/libpcl_common.so;optimized;/usr/lib/libflann_cpp_s.a;debug;/usr/lib/libflann_cpp_s.a;optimized;/usr/lib/libpcl_kdtree.so;debug;/usr/lib/libpcl_kdtree.so;optimized;/usr/lib/libpcl_octree.so;debug;/usr/lib/libpcl_octree.so;optimized;/usr/lib/libpcl_search.so;debug;/usr/lib/libpcl_search.so;optimized;/usr/lib/libpcl_sample_consensus.so;debug;/usr/lib/libpcl_sample_consensus.so;optimized;/usr/lib/libpcl_filters.so;debug;/usr/lib/libpcl_filters.so;optimized;/usr/local/lib/libpcl_ml.so;debug;/usr/local/lib/libpcl_ml.so;/usr/lib/libOpenNI.so;vtkCommon;vtkRendering;vtkHybrid;optimized;/usr/lib/libpcl_io.so;debug;/usr/lib/libpcl_io.so;optimized;/usr/local/lib/libqhullstatic.a;debug;/usr/lib/libqhull.so;optimized;/usr/lib/libpcl_surface.so;debug;/usr/lib/libpcl_surface.so;optimized;/usr/lib/libpcl_features.so;debug;/usr/lib/libpcl_features.so;optimized;/usr/lib/libpcl_geometry.so;debug;/usr/lib/libpcl_geometry.so;optimized;/usr/lib/libpcl_visualization.so;debug;/usr/lib/libpcl_visualization.so;optimized;/usr/lib/libpcl_segmentation.so;debug;/usr/lib/libpcl_segmentation.so;optimized;/usr/lib/libpcl_registration.so;debug;/usr/lib/libpcl_registration.so;optimized;/usr/lib/libpcl_keypoints.so;debug;/usr/lib/libpcl_keypoints.so;optimized;/usr/lib/libpcl_tracking.so;debug;/usr/lib/libpcl_tracking.so;") 

file(GLOB kinectpclviewer_SRC
    "*.h"
    "kinect.cpp"
    )
add_executable(kinectPCLviewer ${kinectpclviewer_SRC})
add_executable(cloud_segment ./cloud_segment.cpp)

target_link_libraries (kinectPCLviewer 
    ${PCL_LIBRARIES}
    )

target_link_libraries (cloud_segment
    ${PCL_LIBRARIES}
    )
