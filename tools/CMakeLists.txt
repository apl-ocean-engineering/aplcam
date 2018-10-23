
# include_directories(
# 	${CMAKE_SOURCE_DIR}/include
# )
#
# link_libraries(
# 		aplcam
# 		${CONAN_LIBS}
# 		${OpenCV_LIBS}
# 		${Boost_LIBRARIES}
# 		${CERES_LIBRARY}
# 		${LevelDB_LIBRARIES}
# 		${GSL_LIBRARIES}
# 		glog
# 		cryptopp )

#	${GSL_LIBRARIES}
#	${Boost_LIBRARIES}
#        ${CERES_LIBRARIES}
#	${KyotoCabinet_LIBRARIES}
#        cryptopp )
#

# fips_begin_app( video_splitter cmdline )
#  	fips_files( video_splitter.cpp )
# fips_end_app()
#
# fips_begin_app( extract_one_frame cmdline )
#  	fips_files( extract_one_frame.cpp )
# fips_end_app()
#
# if( USE_APRILTAGS )
#   fips_begin_app( apriltags_player cmdline )
# 		fips_files( apriltags_player.cpp )
# 		fips_deps( apriltags )
# 	fips_end_app()
# endif()
