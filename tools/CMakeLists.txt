
link_libraries(
		${aplcam_LIBS}
		${OpenCV_LIBS}
		${Boost_LIBRARIES}
		glog
		cryptopp )

#	${GSL_LIBRARIES}
#	${Boost_LIBRARIES}
#        ${CERES_LIBRARIES}
#	${KyotoCabinet_LIBRARIES}
#        cryptopp )
#

add_executable( video_splitter video_splitter.cpp )

add_executable( extract_one_frame extract_one_frame.cpp )

if( USE_APRILTAGS )
  add_executable( apriltags_player apriltags_player.cpp )
endif()
