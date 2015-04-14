add_library( aplcam
        bits_to_hex.cpp
        types.cpp
	board.cpp 
        image.cpp
	video.cpp 
	detection.cpp 
        detection_db.cpp
        my_undistort.cpp
	april_tag_board_generator.cpp 
	april_tag_detection_set.cpp  
	file_utils.cpp
        synchronizer.cpp 
	trendnet_time_code.cpp 
        camera_model.cpp
        pinhole_camera.cpp
        distortion_model.cpp
        angular_polynomial.cpp
        radial_polynomial.cpp
        camera_factory.cpp
        distortion_stereo.cpp
        stereo_calibration.cpp 
        feature_tracker.cpp
        calibration_db.cpp
        calibrator.cpp
	splitter_common.cpp
        video_splitters/video_splitter_opts.cpp )

target_include_directories( aplcam
	PUBLIC ${libaplcam_SOURCE_DIR}/include 
        PUBLIC ${APRILTAGS_INCLUDE_DIR} )

