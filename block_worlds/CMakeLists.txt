cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(block_worlds_visiualization)

#file(GLOB source
    #"*.h"
    #"*.cpp"
    #"../m_util.cpp" 
#)

ADD_EXECUTABLE (block_worlds main.cpp)
# my library
INCLUDE_DIRECTORIES(${M_LIBRARY_INCLUDE})
TARGET_LINK_LIBRARIES(block_worlds ${M_LIBRARY_LIB}/libm_library.a)
#
FIND_PACKAGE(PCL 1.2 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

TARGET_LINK_LIBRARIES (block_worlds
    /usr/lib/libxmlrpc++.so
    /usr/lib/libxmlrpc_client++.so
    /usr/lib/libxmlrpc.so.3
    /usr/lib/libxmlrpc_abyss.so
    /usr/lib/libboost_regex.so
    ${PCL_LIBRARIES})
