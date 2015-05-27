
#include "trendnet_time_code.h"

using namespace cv;

//const Rect timeCodeROI_1920x1080( 20, 8, 227, 14 );

namespace TimeCode_1920x1080 {

Mat timeCodeMask( void )
{
  Mat mask( Mat::zeros( 1080, 1920, CV_8UC1 ) );
  Mat roi( mask, timeCodeROI );
  roi.setTo( 1 );
  return mask;
}

Mat timeCodeUnmask( void )
{
  Mat mask( Mat::ones( 1080, 1920, CV_8UC1 ) );
  Mat roi( mask, timeCodeROI );
  roi.setTo( 0 );
  return mask;
}


}
