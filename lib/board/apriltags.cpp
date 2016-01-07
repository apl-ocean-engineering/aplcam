#ifndef USE_APRILTAGS
#warn "Compiling __FILE__ even though Apriltag support hasn't been configured."
#else

#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <glog/logging.h>

#include "AprilTags/SubtagDetector.h"
// #include "AprilTags/SubtagDetection.h"

#include "board/apriltags.h"
#include "detection/apriltags.h"

namespace AplCam {

  using namespace cv;

  const float AprilTagsBoard::DefaultSubtagThreshold  = 0.1;


  //===========================================================================
  //  AprilTagsBoard
  //===========================================================================

  //const int AprilTagsBoard::_ids[] = { 0,1,2,3,4,5,6,
  //  24,25,26,27,28,29,30,
  //  48,49,50,51,52,53,54,
  //  72,73,74,75,76,77,78,
  //  96,97,98,99,100,101,102 };


  AprilTagsBoard::AprilTagsBoard( int w, int h, float squares,
    const std::string &name,
    bool doSubtags  )
    : Board( APRILTAGS, w, h, squares, name ),
    _tagCode( AprilTags::tagCodes36h11 ),
    _subtagThreshold( doSubtags ? DefaultSubtagThreshold : 1.0)
    {;}

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

      if( _subtagThreshold < 1.0 ) {
        AprilTagsSubtagDetection *detect = attemptSubtagDetection( gray, detections );
        if( detect != NULL ) return detect;
      }

      AprilTagsDetection *detect = new AprilTagsDetection( detections );
      detect->calculateCorners( *this );
      return detect;
    }

    AprilTagsSubtagDetection *AprilTagsBoard::attemptSubtagDetection( const Mat &gray, vector<AprilTags::TagDetection> &detections )
    {
      AprilTags::SubtagDetector subtag( AprilTags::tagCodes36h11 );
      vector<AprilTags::SubtagDetection> subtagDetections;

      for( unsigned int i = 0; i < detections.size(); ++i ){
        float area = detections[i].totalArea();
	float pct = area / (gray.cols * gray.rows);

        LOG(INFO) << "Tag is " << area << " pixels, " << pct*100.0 << " pct of total image area.";
        if( pct <= _subtagThreshold ) {
          LOG(INFO) << "  ... less than threshold " << 100*_subtagThreshold << " pct, skip.";
          continue;
        }

        AprilTags::SubtagDetection det( subtag.detectTagSubstructure( gray, detections[i] ));

        if( det.good ) subtagDetections.push_back( det );
      }

      if( subtagDetections.size() > 0 )
        return new AprilTagsSubtagDetection();

      return NULL;
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
