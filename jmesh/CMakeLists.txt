CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#file(GLOB source
    #"*.cpp"
    #)
INCLUDE_DIRECTORIES("./include")
ADD_LIBRARY(jmesh
        ./src/PRIMITIVES/binTree
		./src/PRIMITIVES/dijkstraGraph
        ./src/PRIMITIVES/heap
        ./src/PRIMITIVES/list
        ./src/PRIMITIVES/clusterGraph
        ./src/PRIMITIVES/graph
        ./src/PRIMITIVES/jqsort
        ./src/PRIMITIVES/matrix
        ./src/MESH_STRUCTURE/point
        ./src/MESH_STRUCTURE/vertex
        ./src/MESH_STRUCTURE/edge
        ./src/MESH_STRUCTURE/triangle
        ./src/MESH_STRUCTURE/tin
        ./src/MESH_STRUCTURE/io
        ./src/MESH_STRUCTURE/checkAndRepair
        ./src/JMESH/jmesh
        )


#TARGET_LINK_LIBRARIES(m_library
    #${USER_LIB}/libOGDF.a
    #/usr/lib/libxmlrpc_abyss.so
    #/usr/lib/libxmlrpc_client++.so
    #/usr/lib/libxmlrpc++.so
    #/usr/lib/libxmlrpc.so.3
    #/usr/lib/libboost_graph.so
    #/usr/lib/libopencv_highgui.so
    #/usr/lib/libxmlrpc_server_abyss++.so
    #/usr/lib/libxmlrpc_server++.so
    #/usr/lib/libboost_system.so
    #/usr/lib/libboost_filesystem.so
    #${LOCAL_LIB}/libopencv_core.so
    #${LOCAL_LIB}/libopencv_imgproc.so
    #/usr/lib/libflann_cpp.so
    #/usr/lib/libboost_regex.so)
