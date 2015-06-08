
#include "detection_set.h"

namespace AplCam {


  static bool RemoveIfNoValidate( Detection *det )
  {
    return det->validate() >= 0;
  }

  int DetectionSet::validate( void )
  {
    DetectionMap::iterator newEnd = std::remove_if( _detections.begin(), _detections.end(), RemoveIfNoValidate );

    int newSize = 0;
    for( DetectionMap::iterator itr = _detections.begin(); itr != newEnd; ++newSize, ++itr ) {;}

    _detections.resize( newSize );

    return size();

  }

}
