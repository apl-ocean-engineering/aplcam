
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
  camera_model.cpp
  pinhole_camera.cpp
  distortion_model.cpp
  angular_polynomial.cpp
  radial_polynomial.cpp
  ceres_radial_polynomial.cpp
  camera_factory.cpp
  distortion_stereo.cpp
  stereo_calibration.cpp 
  motion_model.cpp
  feature_tracker.cpp
  calibration_db.cpp
  splitter_common.cpp
  frame_selector/keyframe_selector.cpp
  calib_frame_selectors/random_frame_selector.cpp 
  calib_frame_selectors/keyframe_frame_selector.cpp )

target_include_directories( aplcam
  PUBLIC ${libaplcam_SOURCE_DIR}/include  )

if( USE_APRILTAGS )
  target_include_directories( aplcam
    PUBLIC ${APRILTAGS_INCLUDE_DIR} )
endif()


