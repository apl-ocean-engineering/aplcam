
## Try pulling in the aplcam as an external project
include( ExternalProject )

if( Apriltags_SOURCE_DIR )
  message( "Using Apriltags from directory at ${Apriltags_SOURCE_DIR}" )
  ExternalProject_Add( apriltags
    DOWNLOAD_COMMAND ""
    SOURCE_DIR ${Apriltags_SOURCE_DIR}
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/apriltags
    CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}"
    BUILD_COMMAND ${CMAKE_COMMAND} --build .
    INSTALL_COMMAND ""
  )
else()
  message( "Using Apriltags from git repo" )
  ExternalProject_Add( apriltags
    GIT_REPOSITORY "https://github.com/amarburg/apriltags.git"
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/apriltags
    CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}"
    BUILD_COMMAND ${CMAKE_COMMAND} --build .
    INSTALL_COMMAND ""
  )
  ## Should get Apriltags from sources...
endif()

ExternalProject_Get_Property( apriltags SOURCE_DIR BINARY_DIR )

set( apriltags_INCLUDE_DIR ${SOURCE_DIR}/include )
add_library( libapriltags STATIC IMPORTED )
set_property( TARGET libapriltags PROPERTY IMPORTED_LOCATION ${BINARY_DIR}/libapriltags.a )

find_package_handle_standard_args( apriltags DEFAULT_MSG apriltags_INCLUDE_DIR )
