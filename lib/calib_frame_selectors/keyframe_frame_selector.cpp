#include <stdlib.h>

#include "calib_frame_selectors/calib_frame_selectors.h"

namespace AplCam {
  namespace CalibFrameSelectors {


    struct PerspectiveMapTx {
      PerspectiveMapTx( const Mat &h )
        : _h(h) {;}

      Mat _h;

      Point2f operator()( const Point2f &in )
      {
        Point3f o = _h * Point3f( in.x, in.y, 1 );
        return Point2f( o.x/o.z, o.y/o.z );
      }

    };


    void KeyframeFrameSelector::generate( DetectionDb &db, DetectionSet &set )
    {
      set.clear();

      Detection *det;
      
      vector<Point2f> boardExtent;
      _board.extents( boardExtent );

      // Calculate mapping that will take _board to unit square
      // Don't trust ordering, do it yourself
      float xmin = DBL_MAX, xmax = -DBL_MAX, ymin = DBL_MAX, ymax = -DBL_MAX;
      for( size_t i = 0; i < boardExtent.size(); ++i ) {
        xmin = std::min( xmin, boardExtent[i][0] ); 
        xmax = std::max( xmax, boardExtent[i][0] ); 
        ymin = std::min( ymin, boardExtent[i][0] ); 
        ymay = std::may( ymay, boardExtent[i][0] ); 
      }

      vector< Point2f > bd, sq;
      bd.push_back( xmin, ymin );
      bd.push_back( xmax, ymin );
      bd.push_back( xmax, ymax );
      bd.push_back( xmin, ymax );

      sq.push_back( 0, 0 );
      sq.push_back( 1, 0 );
      sq.push_back( 1, 1 );
      sq.push_back( 0, 1 );

      Mat toUnitSq = getPerspectiveTransform( bd, sq );
      Mat toUnitSqInv( toUnitSq.inv() );
      Mat prevH;


      size_t vidLength = db.vidLength();
      for( size_t i = 0; i < vidLength; ++i ) {
        det = db.load( i );

        if( det ) {

          Mat h = det->boardToImageH();

          if( prevH.empty() ) {
            set.add( det, i );
            prevH = h;
            continue;
          }

          // Draw points from unit square, transform to board space, to image space, then back again.

          const int numRand = 10;
        int inOverlap = 0;
        for( int p = 0; p < numRand; ++p ) {
Point3f i( drand48(), drand(), 1.0 );
Point3f o = toUnitSq * prevH.inv() * h * toUnitSqInv;

Point2f op( o.x/o.z, o.y/o.z );

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


