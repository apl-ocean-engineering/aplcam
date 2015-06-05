
#include <opencv2/calib3d/calib3d.hpp>
#include "detection.h"
#include "file_utils.h"

using namespace std;
using namespace cv;

using namespace AplCam;

//============================================================================
// HoughCircleDetection
//============================================================================

void HoughCircleDetection::drawCorners( const Board &board, Mat &view ) const
{
  for( size_t i = 0; i < _circles.size() ; ++i ) {
    circle( view, Point( _circles[i][0], _circles[i][1] ), _circles[i][2], Scalar( 0, 0, 255 ), 2 );
  }
}


