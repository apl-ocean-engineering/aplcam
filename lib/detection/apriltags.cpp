
#include <opencv2/calib3d/calib3d.hpp>
#include "detection.h"
#include "file_utils.h"

#include "detection/apriltags.h"
#include "board/apriltags.h"

using namespace std;
using namespace cv;

using namespace AplCam;

#ifdef USE_APRILTAGS

//============================================================================
//  AprilTagDetection
//============================================================================

void AprilTagsDetection::calculateCorners( const AprilTagsBoard &board )
{
  // Go for a simple model here, assume all tags are unique on the board

  points.clear();
  corners.clear();
  ids.clear();

  for( size_t i = 0; i < _det.size(); ++i ) {

    Point2i loc;
    if( board.find( _det[i].id, loc ) ) {
      //cout << "Found  tag id " << _det[i].id << endl;

      points.push_back( Point2f( _det[i].cxy.first, _det[i].cxy.second ) );
      corners.push_back( board.worldLocation( loc ) );
      ids.push_back( _det[i].id );

    } else {
      cerr << "Couldn't find tag \'" << _det[i].id << "\'" << endl;
    }
  }
}

#endif

