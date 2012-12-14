CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
file(GLOB source
    "*.h"
    "*.cpp"
    )

#${M_LIBRARY}/data.cpp
ADD_EXECUTABLE(object_detection ${source})
# my library
INCLUDE_DIRECTORIES(${M_LIBRARY_INCLUDE})
TARGET_LINK_LIBRARIES(object_detection ${M_LIBRARY_LIB}/libm_library.a)
# third party library
INCLUDE_DIRECTORIES(
    ${LOCAL_INCLUDE}/opencv 
    ${USER_INCLUDE})
TARGET_LINK_LIBRARIES(object_detection 
    ${USER_LIB}/libOGDF.a
    /usr/lib/libxmlrpc_abyss.so
    /usr/lib/libxmlrpc_client++.so
    /usr/lib/libxmlrpc++.so
    /usr/lib/libxmlrpc.so.3
    /usr/lib/libboost_graph.so
    ${LOCAL_LIB}/libopencv_highgui.so
    /usr/lib/libxmlrpc_server_abyss++.so
    /usr/lib/libxmlrpc_server++.so
    /usr/lib/libboost_system.so
    ${LOCAL_LIB}/libopencv_core.so
    ${LOCAL_LIB}/libopencv_imgproc.so
    /usr/lib/i386-linux-gnu/libpthread.so
    /usr/lib/libboost_regex.so)
#TARGET_LINK_LIBRARIES(object_detection /usr/lib/qt4/

#INSTALL (TARGETS object_detection
    #RUNTIME DESTINATION ..
    #)
#INSTALL_TARGETS(.. object_detection)
