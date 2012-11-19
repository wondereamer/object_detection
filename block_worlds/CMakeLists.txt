cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(block_worlds_visiualization)

#file(GLOB source
    #"*.h"
    #"*.cpp"
    #"../m_util.cpp" 
#)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${M_LIBRARY})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (block_worlds
    main.cpp
    ${M_LIBRARY}/m_util.cpp
    ${M_LIBRARY}/data.cpp
    )
target_link_libraries (block_worlds
    /usr/lib/libxmlrpc++.so
    /usr/lib/libxmlrpc_client++.so
    /usr/lib/libxmlrpc.so.3
    /usr/lib/libxmlrpc_abyss.so
    /usr/lib/libboost_regex.so
    ${PCL_LIBRARIES})
