
if( USE_APRILTAGS )
  message( INFO " USE_APRILTAGS set" )
  set( APRILTAG_SRCS
    april_tag_board_generator.cpp
    april_tag_detection_set.cpp
    board/apriltags.cpp
    detection/apriltags.cpp )
else()
  message( INFO " USE_APRILTAGS not set" )
endif()

add_library( aplcam
  bits_to_hex.cpp
  types.cpp
  board.cpp
  board/circle.cpp
  image.cpp
  video.cpp
  detection.cpp
  detection/circle.cpp
  detection_db.cpp
  detection_set.cpp
  my_undistort.cpp
  ${APRILTAG_SRCS}
  file_utils.cpp
  synchronizer.cpp
  trendnet_time_code.cpp
  distortion/camera_model.cpp
  distortion/pinhole_camera.cpp
  distortion/distortion_model.cpp
  distortion/angular_polynomial.cpp
  distortion/radial_polynomial.cpp
  distortion/opencv_radial_polynomial.cpp
  distortion/ceres_radial_polynomial.cpp
  distortion/camera_factory.cpp
  distortion/distortion_stereo.cpp
  distortion/stereo_calibration.cpp
  motion_model.cpp
  feature_tracker.cpp
  calibration_db.cpp
  splitter_common.cpp
  hough_circles.cpp
  image_accumulator.cpp
  sonar_pose.cpp
  frame_selector/keyframe_selector.cpp
  calib_frame_selectors/random_frame_selector.cpp
  calib_frame_selectors/keyframe_frame_selector.cpp )

target_include_directories( aplcam
	PUBLIC	${aplcam_INCLUDE_DIRS} )
