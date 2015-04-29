#include <stdlib.h>

#include "calib_frame_selectors/calib_frame_selectors.h"

using namespace cv;

namespace AplCam {
  namespace CalibFrameSelectors {


    struct PerspectiveMapTx {
      PerspectiveMapTx( const Matx33d &h )
        : _h(h) {;}

      Matx33d _h;

      Point2f operator()( const Point2f &in )
      {
        Vec3f o = _h * Vec3f( in.x, in.y, 1 );
        return Point2f( o[0]/o[2], o[1]/o[2] );
      }

    };


    void KeyframeFrameSelector::generate( DetectionDb &db, DetectionSet &set )
    {
      Detection *det;
      
      ObjectPointsVec boardExtent;
      _board.extents( boardExtent );

      // Calculate mapping that will take _board to unit square
      // Don't trust ordering, do it yourself
      float xmin = DBL_MAX, xmax = -DBL_MAX, ymin = DBL_MAX, ymax = -DBL_MAX;
      for( size_t i = 0; i < boardExtent.size(); ++i ) {
        xmin = std::min( xmin, boardExtent[i][0] ); 
        xmax = std::max( xmax, boardExtent[i][0] ); 
        ymin = std::min( ymin, boardExtent[i][1] ); 
        ymax = std::max( ymax, boardExtent[i][1] ); 
      }

      vector< Point2f > bd, sq;
      bd.push_back( Point2f( xmin, ymin ) );
      bd.push_back( Point2f( xmax, ymin ) );
      bd.push_back( Point2f( xmax, ymax ) );
      bd.push_back( Point2f( xmin, ymax ) );

      sq.push_back( Point2f( 0, 0 ) );
      sq.push_back( Point2f( 1, 0 ) );
      sq.push_back( Point2f( 1, 1 ) );
      sq.push_back( Point2f( 0, 1 ) );

      Matx33d toUnitSq = getPerspectiveTransform( bd, sq );
      Matx33d toUnitSqInv( toUnitSq.inv() );
      Matx33d prevH;


      bool first = true;
      size_t vidLength = db.vidLength();
      for( size_t i = 0; i < vidLength; ++i ) {
        det = db.load( i );

        if( det ) {

          Matx33d h = det->boardToImageH();

          if( first ) {
            set.addDetection( det, i );
            prevH = h;
            first = false;
            continue;
          }

          // Draw points from unit square, transform to board space, to image space, then back again.

          const int numRand = 10;
          int inOverlap = 0;
          for( int p = 0; p < numRand; ++p ) {
            Vec3f i( drand48(), drand48(), 1.0 );
            Vec3f o = toUnitSq * prevH.inv() * h * toUnitSqInv * i;
            Point2f op( o[0]/o[2], o[1]/o[2] );

            cout << op << endl;
            if( op.x >= 0.0 && op.x < 1.0 && op.y >= 0.0 && op.y < 1.0 ) ++inOverlap;
          }

          float overlap = inOverlap * 1.0 / numRand;

        cout << "Overlap " << overlap << endl;

          // Only delete if not added to a set
          delete det;
        }
      }

    }

  }
}


