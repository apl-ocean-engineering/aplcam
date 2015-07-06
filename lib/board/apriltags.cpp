#ifndef USE_APRILTAGS
#warn "Compiling __FILE__ even though Apriltag support hasn't been configured."
#else

#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "board/apriltags.h"
#include "detection/apriltags.h"

#include <glog/logging.h>


namespace AplCam {

using namespace cv;


//===========================================================================
//  AprilTagsBoard
//===========================================================================

//const int AprilTagsBoard::_ids[] = { 0,1,2,3,4,5,6,
//  24,25,26,27,28,29,30,
//  48,49,50,51,52,53,54,
//  72,73,74,75,76,77,78,
//  96,97,98,99,100,101,102 };

void AprilTagsBoard::loadCallback( FileStorage &fs )
{
  fs["ids"] >> _ids;
}

Detection *AprilTagsBoard::detectPattern( const cv::Mat &img )
{
  AprilTags::TagDetector tagDetector( _tagCode );

  Mat gray;
  ensureGrayscale( img, gray );

  vector<AprilTags::TagDetection> detections = tagDetector.extractTags(gray);
  LOG(INFO) << "Detected " << detections.size() << " AprilTags";

  AprilTagsDetection *detect = new AprilTagsDetection( detections );
  detect->calculateCorners( *this );
  return detect;
}

bool AprilTagsBoard::find( const int id, cv::Point2i &xy  ) const
{
  for( int x = 0; x < width; ++x )
    for( int y = 0; y < height; ++y )
      if( _ids.at<int>(y,x) == id )
      {
        xy.x = x; xy.y = y;
        return true;
      }

  return false;
}

std::vector< int > AprilTagsBoard::ids( void )
{
  vector< int > out;
  for( int x = 0; x < width; ++x )
    for( int y = 0; y < height; ++y )
      out.push_back( _ids.at< int >( y, x ) );

  return out;
}

}

#endif
