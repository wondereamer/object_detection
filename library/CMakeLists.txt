CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#file(GLOB source
    #"*.cpp"
    #)

ADD_LIBRARY(m_library
    m_algorithm
    m_geometry
    m_opencv
    m_util
    data
    vertex_color)
INCLUDE_DIRECTORIES(
    ${LOCAL_INCLUDE}/opencv 
    ${USER_INCLUDE}
    ${GLOBAL_INCLUDE}
    )
TARGET_LINK_LIBRARIES(m_library
    ${USER_LIB}/libOGDF.a
    /usr/lib/libxmlrpc_abyss.so
    /usr/lib/libxmlrpc_client++.so
    /usr/lib/libxmlrpc++.so
    /usr/lib/libxmlrpc.so.3
    /usr/lib/libboost_graph.so
    /usr/lib/libopencv_highgui.so
    /usr/lib/libxmlrpc_server_abyss++.so
    /usr/lib/libxmlrpc_server++.so
    /usr/lib/libboost_system.so
    ${LOCAL_LIB}/libopencv_core.so
    ${LOCAL_LIB}/libopencv_imgproc.so
    /usr/lib/libflann_cpp.so
    /usr/lib/libboost_regex.so)
