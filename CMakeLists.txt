if (NOT FIPS_IMPORT)
  cmake_minimum_required(VERSION 3.5.0)

  get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
  include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

  fips_setup(PROJECT aplcam)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package( OpenCV REQUIRED core imgproc calib3d features2d highgui )
if( OpenCV_VERSION_MAJOR VERSION_LESS "3.0.0" )
  add_definitions( -DOPENCV_2 )
else()
  add_definitions( -DOPENCV_3 )
endif()

#find_package( PCL 1.2 REQUIRED )
find_package( Ceres REQUIRED )

# Packages supported by local Find*.cmake files
find_package( GSL REQUIRED )
find_package( Eigen3 REQUIRED )

## As suggested, always build a local copy of GTest

#
#
#
## KyotoCabinet stuff
find_path(KyotoCabinet_INCLUDE_DIR kcdb.h
 HINTS ${PC_KyotoCabinet_INCLUDEDIR} ${PC_KyotoCabinet_INCLUDE_DIRS})

find_library(KyotoCabinet_LIBRARY NAMES libkyotocabinet kyotocabinet
      HINTS /usr/lib /usr/lib/x86_64-linux-gnu/
      ${PC_KyotoCabinet_LIBDIR} ${PC_KyotoCabinet_LIBRARY_DIRS})

set(KyotoCabinet_LIBS ${KyotoCabinet_LIBRARY})
set(KyotoCabinet_INCLUDE_DIRS ${KyotoCabinet_INCLUDE_DIR})

find_package( Boost REQUIRED COMPONENTS thread system filesystem )
set( Boost_USE_MULTITHREADED ON )

add_definitions( -std=c++11 -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_ALIGN )

option( USE_TBB "Use TBB" ON )
if( USE_TBB )
  add_definitions( -DUSE_TBB )   # -DTBB_USE_DEBUG=1
endif()

option( USE_APRILTAGS "Use AprilTags" ON )
if( USE_APRILTAGS )
  ## In the transition to Conan, force use of Apriltags for now...
  add_definitions( -DUSE_APRILTAGS )
endif()


fips_include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  include )

link_directories( /usr/local/lib )



OPTION( BUILD_SHARED_LIBS "Build shared libraries" ON )


fips_add_subdirectory(lib)
fips_add_subdirectory(tools)



if( NOT FIPS_IMPORT )

  # if( FIPS_UNITTESTS )
  #   add_subdirectory( test/unit )
  # endif()

  fips_finish()
endif()
