#ifndef __APRILTAGS_DETECTION_H__
#define __APRILTAGS_DETECTION_H__

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "board/apriltags.h"
#include "distortion_model.h"

#include <kchashdb.h>
#include <AprilTags/TagDetection.h>

#include "detection.h"

#ifdef USE_APRILTAGS

namespace AplCam {
struct AprilTagsDetection : public Detection
  {
    AprilTagsDetection( vector< AprilTags::TagDetection > det )
        : Detection(), _det(det) {;}

    vector< AprilTags::TagDetection > _det;

    virtual void calculateCorners( const AprilTagsBoard &board );

  virtual Detection::Validate_Return_Code validate( void );

  };
}
#endif

#endif
