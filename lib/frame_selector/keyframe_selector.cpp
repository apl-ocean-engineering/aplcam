#include <opencv2/features2d.hpp>

#include "AplCam/frame_selector/frame_selector.h"

namespace AplCam {


  KeyframeSelector::KeyframeSelector( void )
  {;}


  bool KeyframeSelector::process( Mat &img )
  {
    if( _kfKeyPoints.empty() ) {
      FAST( img, _kfKeyPoints, true );
      return true;
    }



    return false;
  }

};
