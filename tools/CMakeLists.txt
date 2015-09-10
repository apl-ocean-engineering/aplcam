
include_directories( "${CMAKE_SOURCE_DIR}/include"
		      ${TCLAP_INCLUDE_DIR}  )

link_libraries( ${OpenCV_LIBS}
							  aplcam
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
  target_include_directories( apriltags_player PUBLIC ${apriltags_INCLUDE_DIR} )
  target_link_libraries( apriltags_player libapriltags ${OpenCV_LIBS} )
endif()
