CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#ADD_LIBRARY (builtinreqs SHARED
#BuiltinRequestsModule
#Request
#DataRequest
#ExitRequest
#SleepRequest
#HelpRequest
#LoadRequest
#SaveRequest
#LoadModuleRequest
#ListRequest
#ShutdownRequest
#UnloadModuleRequest
#)

#TARGET_LINK_LIBRARIES(builtinreqs atomspace util)

#add_definitions("-std=c99 -g -w -Winline")

INCLUDE_DIRECTORIES(
    ${OPENCOG_LOCATION}
    ${M_LIBRARY}
    /usr/include/opencv
    )

ADD_EXECUTABLE(data_server
    main
    ${M_LIBRARY}/m_util
    )

TARGET_LINK_LIBRARIES(data_server
    ${OPENCOG_LOCATION}/build/opencog/server/libserver.so
    ${OPENCOG_LOCATION}/build/opencog/guile/libsmob.so
    ${OPENCOG_LOCATION}/build/opencog/atomspace/libatomspace.so
    ${OPENCOG_LOCATION}/build/opencog/util/libutil.so
    /usr/lib/libboost_filesystem.so
    /usr/lib/libboost_thread.so
    /usr/lib/libboost_system.so
    /usr/lib/libxmlrpc++.so
    /usr/lib/libxmlrpc.so.3
    /usr/lib/libxmlrpc_client++.so
    /usr/lib/libxmlrpc_server_abyss++.so
    /usr/lib/libxmlrpc_server++.so
    /usr/lib/libopencv_highgui.so

    #${Boost_FILESYSTEM_LIBRARY}
    #${Boost_THREAD_LIBRARY}
    )

