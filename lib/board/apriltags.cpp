
#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "board/apriltags.h"
#include "detection/apriltags.h"

#ifdef USE_APRILTAGS

namespace AplCam {


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

Detection *AprilTagsBoard::detectPattern( const cv::Mat &img, vector< cv::Point2f > &pointbuf )
{
  AprilTags::TagDetector tagDetector( _tagCode );

  // Automatically convert to gray --- somewhat ambivalent about this
  // but otherwise require use to know if detector takes color or gray images
  Mat gray;
  if( img.channels() != 1 ) {
    cvtColor( img, gray, CV_BGR2GRAY );
  } else {
    gray = img;
  }

  vector<AprilTags::TagDetection> detections = tagDetector.extractTags(gray);
  //cout << "found " << detections.size() << " tags:" << endl;

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
