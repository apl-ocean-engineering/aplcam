
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "detection/circle.h"

using namespace std;

using namespace AplCam;

using cv::Mat;
using cv::Scalar;
using cv::Point;

//============================================================================
// CircleDetection
//============================================================================

void CircleDetection::drawCorners( const Board &board, Mat &view ) const
{
  for( size_t i = 0; i < _circles.size() ; ++i ) {
    cv::circle( view, Point( _circles[i][0], _circles[i][1] ), _circles[i][2], Scalar( 0, 0, 255 ), 1 );
  }
}


