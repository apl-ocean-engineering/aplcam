#ifndef __CIRCLE_DETECTION_H__
#define __CIRCLE_DETECTION_H__

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "detection.h"

#include <kchashdb.h>

namespace AplCam {

using std::string;

struct CircleDetection : public Detection 
{
  CircleDetection( vector< cv::Vec3f > circles )
    : _circles( circles ) 
  {
    corners.clear();
    corners.push_back( AplCam::ObjectPoint(0,0) );
  }


  virtual void drawCorners(  const Board &board, cv::Mat &view ) const;

  vector< cv::Vec3f > _circles;
};

}

#endif
