CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

find_package(PCL 1.6 REQUIRED)

#include_directories("/usr/include/pcl-1.6")
#include_directories("/usr/include/eigen3") 
#include_directories("/usr/include/openni") 
#link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
link_directories(${GLOBAL_LIB})
#set(PCL_LIBRARIES "/usr/lib/libboost_system-mt.so;/usr/lib/libboost_filesystem-mt.so;/usr/lib/libboost_thread-mt.so;/usr/lib/libboost_date_time-mt.so;/usr/lib/libboost_iostreams-mt.so;/usr/lib/libboost_mpi-mt.so;/usr/lib/libboost_serialization-mt.so;optimized;/usr/lib/libpcl_common.so;debug;/usr/lib/libpcl_common.so;optimized;/usr/lib/libflann_cpp_s.a;debug;/usr/lib/libflann_cpp_s.a;optimized;/usr/lib/libpcl_kdtree.so;debug;/usr/lib/libpcl_kdtree.so;optimized;/usr/lib/libpcl_octree.so;debug;/usr/lib/libpcl_octree.so;optimized;/usr/lib/libpcl_search.so;debug;/usr/lib/libpcl_search.so;optimized;/usr/lib/libpcl_sample_consensus.so;debug;/usr/lib/libpcl_sample_consensus.so;optimized;/usr/lib/libpcl_filters.so;debug;/usr/lib/libpcl_filters.so;optimized;/usr/local/lib/libpcl_ml.so;debug;/usr/local/lib/libpcl_ml.so;/usr/lib/libOpenNI.so;vtkCommon;vtkRendering;vtkHybrid;optimized;/usr/lib/libpcl_io.so;debug;/usr/lib/libpcl_io.so;optimized;/usr/local/lib/libqhullstatic.a;debug;/usr/lib/libqhull.so;optimized;/usr/lib/libpcl_surface.so;debug;/usr/lib/libpcl_surface.so;optimized;/usr/lib/libpcl_features.so;debug;/usr/lib/libpcl_features.so;optimized;/usr/lib/libpcl_geometry.so;debug;/usr/lib/libpcl_geometry.so;optimized;/usr/lib/libpcl_visualization.so;debug;/usr/lib/libpcl_visualization.so;optimized;/usr/lib/libpcl_segmentation.so;debug;/usr/lib/libpcl_segmentation.so;optimized;/usr/lib/libpcl_registration.so;debug;/usr/lib/libpcl_registration.so;optimized;/usr/lib/libpcl_keypoints.so;debug;/usr/lib/libpcl_keypoints.so;optimized;/usr/lib/libpcl_tracking.so;debug;/usr/lib/libpcl_tracking.so;") 
file(GLOB source
    "*.h"
    "*.cpp"
    )

#${M_LIBRARY}/data.cpp
ADD_EXECUTABLE(object_detection ${source})
# my library
INCLUDE_DIRECTORIES(${M_LIBRARY_INCLUDE})
include_directories("../jmesh/include")
link_directories("/usr/local/lib")
link_directories("/usr/lib/i386-linux-gnu")

# Check for boost. We need dynamic-linked, threaded libs by default.
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREADED ON)

# Required boost packages
# hypertable requires boost iostreams
# boost-1.49 no longer has a libboost_iostreams
#FIND_PACKAGE(Boost 1.45 COMPONENTS date_time filesystem regex serialization signals system thread REQUIRED)

## Optional boost packages; can build without these.
## Arghhh. Except cmake is treating these as required, not optional. #$%**&
#FIND_PACKAGE(Boost 1.45 COMPONENTS python program_options QUIET)
#FIND_PACKAGE(Boost 1.45 COMPONENTS math_c99 QUIET)
#IF(Boost_MATH_C99_FOUND)
	#MESSAGE(STATUS "Found Boost::math")
#ELSE(Boost_MATH_C99_FOUND)
	#MESSAGE(STATUS "Boost math missing: needed to run combo learning tests.")
#ENDIF(Boost_MATH_C99_FOUND)

TARGET_LINK_LIBRARIES(object_detection ${M_LIBRARY_LIB}/libm_library.a)
# third party library
INCLUDE_DIRECTORIES(
    ${LOCAL_INCLUDE}/opencv 
    ${USER_INCLUDE})
TARGET_LINK_LIBRARIES(object_detection 
    ${USER_LIB}/libOGDF.a
    libxmlrpc_abyss.so
    /usr/lib/libxmlrpc_client++.so
    /usr/lib/libxmlrpc++.so
    /usr/lib/libxmlrpc.so.3
    /usr/lib/libboost_graph.so
    libopencv_highgui.so
    /usr/lib/libxmlrpc_server_abyss++.so
    /usr/lib/libxmlrpc_server++.so
    /usr/lib/libboost_system.so
    libopencv_core.so
    libopencv_imgproc.so
    /usr/lib/i386-linux-gnu/libpthread.so
    /usr/local/lib/libhdf5.so
    /usr/lib/libflann_cpp.so
    /usr/lib/libboost_regex.so 
    ${JMesh}/libjmesh.a
    /usr/lib/i386-linux-gnu/libXi.so
    ${Boost_GRAPH_LIBRARY}
    ${PCL_LIBRARIES})
message("${PCL_LIBRARIES}")
#TARGET_LINK_LIBRARIES(object_detection /usr/lib/qt4/

#INSTALL (TARGETS object_detection
    #RUNTIME DESTINATION ..
    #)
#INSTALL_TARGETS(.. object_detection)
