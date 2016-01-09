
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

  // class AprilTagsDetection;

  class AprilTagsBoard : public Board {
   public:

    AprilTagsBoard( const Mat &ids, float squares, const std::string &name,  bool doSubtags = false );

    virtual Detection *detectPattern( const cv::Mat &gray );
    Detection *attemptSubtagDetection( const cv::Mat &gray, std::vector<AprilTags::TagDetection> &detections );

    virtual std::vector< int > ids( void );

    bool find( const int id, cv::Point2i &xy  ) const;
    ObjectPoint worldLocation( const int id ) const;

    float setSubtagMinSize( float x ) { return _subtagMinSize = x; }
    float subtagMinSize( void ) const { return _subtagMinSize; }

    static AprilTagsBoard *Load( cv::FileStorage &fs, const string &name );

    void setTagSize( float width, float height = -1 );
    void setBlackBorder( unsigned int border );

   protected:

    // virtual void loadCallback( cv::FileStorage &fs );

   private:

    cv::Mat _ids;
    AprilTags::TagCodes _tagCode;
    int _subtagMinSize;

    unsigned int _blackBorder;
    cv::Size2f _tagSize;
  };

}
#endif


#endif
