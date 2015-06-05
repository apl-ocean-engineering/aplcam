
#ifndef __APRILTAG_BOARD_H__
#define __APRILTAG_BOARD_H__

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

class AprilTagsBoard : public Board {
 public:

  AprilTagsBoard( int w, int h, float squares, const std::string &name )
      : Board(  APRILTAGS, w, h, squares, name ), _tagCode( AprilTags::tagCodes36h11 )
  {;} 

  virtual Detection *detectPattern( const cv::Mat &gray, vector< cv::Point2f > &pointbuf );

  virtual std::vector< int > ids( void );

  bool find( const int id, cv::Point2i &xy  ) const;

 protected:

  virtual void loadCallback( cv::FileStorage &fs );

 private:

  cv::Mat _ids;

  AprilTags::TagCodes _tagCode;
};

}
#endif


#endif
