CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
file(GLOB source
    "*.h"
    "*.cpp"
    )

#${M_LIBRARY}/data.cpp
ADD_EXECUTABLE(test_module ${source})
# my library
INCLUDE_DIRECTORIES(${M_LIBRARY_INCLUDE})
TARGET_LINK_LIBRARIES(test_module ${M_LIBRARY_LIB}/libm_library.a)
# third party library
INCLUDE_DIRECTORIES(
    ${LOCAL_INCLUDE}/opencv 
    ${USER_INCLUDE})
TARGET_LINK_LIBRARIES(test_module 
    ${USER_LIB}/libOGDF.a
    /usr/lib/i386-linux-gnu/libpthread.so)
    #/usr/lib/libxmlrpc_abyss.so
    #/usr/lib/libxmlrpc_client++.so
    #/usr/lib/libxmlrpc++.so
    #/usr/lib/libxmlrpc.so.3
    #/usr/lib/libboost_graph.so
    #/usr/lib/libopencv_highgui.so
    #/usr/lib/libxmlrpc_server_abyss++.so
    #/usr/lib/libxmlrpc_server++.so
    #/usr/lib/libboost_system.so
    #/usr/lib/libopencv_core.so
    #/usr/lib/libopencv_imgproc.so
    #/usr/lib/libboost_regex.so)
#TARGET_LINK_LIBRARIES(test_module /usr/lib/qt4/

#INSTALL (TARGETS test_module
    #RUNTIME DESTINATION ..
    #)
#INSTALL_TARGETS(.. test_module)
