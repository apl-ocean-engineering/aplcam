#pragma once

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "AplCam/board/apriltags.h"

#include <AprilTags/TagDetection.h>

#include "AplCam/detection/detection.h"

#ifdef USE_APRILTAGS

namespace AplCam {

  struct AprilTagsDetection : public Detection
  {
    AprilTagsDetection( vector< AprilTags::TagDetection > det, ObjectPointsVec worldLocations );

    vector< AprilTags::TagDetection > _det;

//    virtual void calculateCorners( const AprilTagsBoard &board );

    virtual Detection::Validate_Return_Code validate( void );

  };

}

#endif
