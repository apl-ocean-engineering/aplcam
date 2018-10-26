#pragma once

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "AplCam/detection/detection.h"

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
