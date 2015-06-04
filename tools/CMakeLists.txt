
include_directories( "${CMAKE_SOURCE_DIR}/include" 
		      ${tclap_INCLUDE_DIR}  )

link_libraries( ${OpenCV_LIBS} ${aplcam_LIBS} ${Boost_LIBRARIES} 
                glog cryptopp )

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
  target_link_libraries( apriltags_player ${APRILTAGS_LIBS} ${OpenCV_LIBS} )
endif()


