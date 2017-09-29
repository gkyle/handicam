cmake_minimum_required (VERSION 2.8.11)

set (CMAKE_MODULE_PATH "./cmake")
set (CMAKE_CXX_STANDARD 11)
set (PROJ_NAME registration)
project(${PROJ_NAME})

FIND_PACKAGE(OpenCV)
FIND_PACKAGE(glog 0.3.5 REQUIRED)
FIND_PACKAGE(V4L2 REQUIRED)

ADD_EXECUTABLE(stitch_stream
  util.hpp
  markers.hpp
  stitcher.hpp
  config.hpp
  util.cpp
  markers.cpp
  stitcher.cpp
  config.cpp
  stitch_stream.cpp)
TARGET_LINK_LIBRARIES(stitch_stream ${OpenCV_LIBS} glog::glog ${V4L2_LIBRARY})

ADD_EXECUTABLE(match_stream
  util.hpp
  markers.hpp
  stitcher.hpp
  config.hpp
  source.hpp
  grid.hpp
  util.cpp
  markers.cpp
  stitcher.cpp
  config.cpp
  source.cpp
  grid.cpp
  match_stream.cpp)
TARGET_LINK_LIBRARIES(match_stream ${OpenCV_LIBS} glog::glog ${V4L2_LIBRARY})

ADD_EXECUTABLE(capture
  util.hpp
  config.hpp
  markers.hpp
  stitcher.hpp
  util.cpp
  config.cpp
  markers.cpp
  stitcher.cpp
  capture.cpp)
TARGET_LINK_LIBRARIES(capture ${OpenCV_LIBS} glog::glog ${V4L2_LIBRARY})

