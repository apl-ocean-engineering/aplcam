
#ifndef __APRILTAGS_BOARD_H__
#define __APRILTAGS_BOARD_H__

#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "types.h"

#ifdef USE_APRILTAGS

#include "board.h"

#include <AprilTags/TagDetector.h>
#include <AprilTags/TagFamily.h>
#include <AprilTags/Tag36h11.h>


namespace AplCam {

  using AplCam::ObjectPointsVec;
  using AplCam::ImagePointsVec;

  class AprilTagsSubtagDetection;
  class AprilTagsDetection;

  class AprilTagsBoard : public Board {
   public:

    static const float DefaultSubtagThreshold;

    AprilTagsBoard( int w, int h, float squares, const std::string &name, bool doSubtags = false );

    virtual Detection *detectPattern( const cv::Mat &gray );
    AprilTagsSubtagDetection *attemptSubtagDetection( const cv::Mat &gray, std::vector<AprilTags::TagDetection> &detections );

    virtual std::vector< int > ids( void );

    bool find( const int id, cv::Point2i &xy  ) const;

    float setSubtagThreshold( float x ) { return _subtagThreshold = x; }
    float subtagThreshold( void ) const { return _subtagThreshold; }

   protected:

    virtual void loadCallback( cv::FileStorage &fs );

   private:

    cv::Mat _ids;
    AprilTags::TagCodes _tagCode;
    float _subtagThreshold;
  };

}
#endif


#endif
