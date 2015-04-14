
include_directories( "${CMAKE_SOURCE_DIR}/include" 
		      ${tclap_INCLUDE_DIR}  )

link_libraries( ${OpenCV_LIBS} ${aplcam_LIBS} ${Boost_LIBRARIES} )

#	${GSL_LIBRARIES}
#	${Boost_LIBRARIES}
#        ${CERES_LIBRARIES}
#	${KyotoCabinet_LIBRARIES}
#        cryptopp )
#

if( USE_APRILTAGS )
  link_libraries( ${APRILTAGS_LIBS} )
endif()


add_executable( video_splitter video_splitter.cpp )
target_link_libraries( video_splitter glog cryptopp )
