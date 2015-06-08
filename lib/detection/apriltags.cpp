#ifndef USE_APRILTAGS
#warn "Compiling __FILE__ even though AprilTags support isn't selected"
#else

#include <opencv2/calib3d/calib3d.hpp>
#include "detection.h"
#include "file_utils.h"

#include "detection/apriltags.h"
#include "board/apriltags.h"

#include <glog/logging.h>

using namespace std;
using namespace cv;



namespace AplCam {

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
        LOG(ERROR) << "Couldn't find tag \'" << _det[i].id << "\'" << endl;
      }
    }
  }


  typedef vector< std::array< size_t, 3 > > IdArray;

  static void buildComboArray( IdArray &arr, size_t maxVal )
  {
    arr.resize( maxVal * (maxVal-1) * (maxVal-2) );

    size_t idx = 0;

    for( size_t i = 0; i < maxVal; ++i ) {
      for( size_t j = 0; j < maxVal; ++i ) {
        if( i == j ) continue;

        for( size_t k = 0; k < maxVal; ++k ) {
          if( k == i || k == j ) continue;

          //std::array<size_t,3> a(i,j,k);
          arr[ idx++ ] = std::array<size_t,3>{ {i,j,k} };

        }
      }
    }
  }

  Detection::Validate_Return_Code AprilTagsDetection::validate( void )
  {
    size_t sz( size() );

    if( sz < 4 ) { return NOT_ENOUGH_POINTS; }

    IdArray combos;
    buildComboArray( combos, sz );

    for( IdArray::iterator itr = combos.begin(); itr != combos.end(); ++itr ) {
      LOG(INFO) << "Trying: " << (*itr)[0] << " " << (*itr)[1] << " " << (*itr)[2] << endl;


    }

  }

}

#endif

