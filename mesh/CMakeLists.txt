### ---[ PCL global CMake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
project(triangle)
add_definitions("-std=gnu++0x -Winline")

find_package(PCL 1.2 REQUIRED)
#todo ... there is two version pcl installed
file(GLOB files
    "*.h"
    "*.cpp" 
)
#find_package(Qt4 4.4.3 COMPONENTS QtCore QtGui QtXml REQUIRED )
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED )
if (QT4_FOUND)
    message(STATUS "Qt4 found!") 
  include(${QT_USE_FILE})
endif (QT4_FOUND)

include_directories("./coin_include")
include_directories("./include")
link_directories("/usr/local/lib")
link_directories("/usr/lib/i386-linux-gnu")
link_directories("./lib")
add_executable(efp ${files})

# Check for boost. We need dynamic-linked, threaded libs by default.
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREADED ON)

# Required boost packages
# hypertable requires boost iostreams
# boost-1.49 no longer has a libboost_iostreams
FIND_PACKAGE(Boost 1.45 COMPONENTS date_time filesystem regex serialization signals system thread REQUIRED)

# Optional boost packages; can build without these.
# Arghhh. Except cmake is treating these as required, not optional. #$%**&
FIND_PACKAGE(Boost 1.45 COMPONENTS python program_options QUIET)
FIND_PACKAGE(Boost 1.45 COMPONENTS math_c99 QUIET)

IF(Boost_MATH_C99_FOUND)
	MESSAGE(STATUS "Found Boost::math")
ELSE(Boost_MATH_C99_FOUND)
	MESSAGE(STATUS "Boost math missing: needed to run combo learning tests.")
ENDIF(Boost_MATH_C99_FOUND)

target_link_libraries (efp
    libCoin.so
    libjmesh.a
    libSoQt.so
    /usr/lib/i386-linux-gnu/libXi.so
    ${QT_LIBRARIES}
    ${Boost_GRAPH_LIBRARY})
