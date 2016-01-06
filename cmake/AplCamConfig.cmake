

if (NOT aplcam_SOURCE_DIR)
	message( FATAL_ERROR "Hmm, aplcam_SOURCE_DIR not set" )
endif ()

## AplCam has external dependencies
list(APPEND CMAKE_MODULE_PATH "${aplcam_SOURCE_DIR}/cmake")
find_package( GSL REQUIRED )
find_package( OpenCV REQUIRED )
find_package( Ceres REQUIRED )
find_package( Boost REQUIRED COMPONENTS thread system filesystem )
set( Boost_USE_MULTITHREADED ON )

# if( USE_APRILTAGS )
#   message("Find_package apriltags in AplCamConfig.txt")
#   find_package( Apriltags REQUIRED )
# 	set( apriltags_LIBS libapriltags )
#
# 	message("Apriltags_SOURCE_DIR is ${Apriltags_SOURCE_DIR}")
# 	# set( Apriltags_SOURCE_DIR ${Apriltags_SOURCE_DIR} CACHE FILEPATH "Location of Apriltags" )
# 	# set( ENV{APRILTAGS_SOURCE_DIR} ${Apriltags_SOURCE_DIR} )
# endif()

ExternalProject_Get_Property( aplcam SOURCE_DIR BINARY_DIR )
add_library( libaplcam STATIC IMPORTED )
set_property( TARGET libaplcam PROPERTY IMPORTED_LOCATION ${BINARY_DIR}/lib/libaplcam.a )

set( aplcam_INCLUDE_DIRS
     ${SOURCE_DIR}/include
     ${apriltags_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS}
		 ${EIGEN_INCLUDE_DIR}
		 ${GSL_INCLUDE_DIRS}
		 ${CERES_INCLUDE_DIRS} )

set( aplcam_LIBS
     libaplcam
     ${apriltags_LIBS}
     cryptopp
     ${OpenCV_LIBS}
     ${GSL_LIBRARIES}
     ${Boost_LIBRARIES}
 		 ${CERES_LIBRARIES}
		 gtest )


find_package_handle_standard_args( aplcam DEFAULT_MSG aplcam_INCLUDE_DIRS aplcam_LIBS )
