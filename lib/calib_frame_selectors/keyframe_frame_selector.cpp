#include <stdlib.h>

#include "calib_frame_selectors/calib_frame_selectors.h"

#define DO_DRAW

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

      cout << xmin << " " << xmax << " : " << ymin << " " << ymax << endl;

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
      Matx33d prevHinv;

#ifdef DO_DRAW
      const string KeyframeDebugWindowName = "keyframe_frame_selector_debug";
        namedWindow( KeyframeDebugWindowName );
        ImagePointsVec prevPts;
#endif

      bool first = true;
      size_t vidLength = db.vidLength();
      for( size_t i = 0; i < vidLength; ++i ) {
        det = db.load( i );

        if( det ) {

          Matx33d h = det->boardToImageH();

#ifdef DO_DRAW
        Mat canvas( Mat::zeros( 1080, 1920, CV_8UC3 ) ) ;

        // Current points
        for( size_t c = 0; c < det->points.size(); ++c ) {
          Point pt( det->points[c][0], det->points[c][1] );
          circle( canvas, pt, 5, Scalar(0,0,255), 1 );
        }

        for( size_t c = 0; c < prevPts.size(); ++c ) {
          Point pt( prevPts[c][0], prevPts[c][1] );
          circle( canvas, pt, 5, Scalar(255,0,0), 1 );
        }

        // Draw the bounding rectangles for the two boards
        vector< Point2f > bbPrev, bbThis;
        for( size_t c = 0; c < sq.size(); ++c ) {
          Vec3f s( sq[c].x, sq[c].y, 1.0 );
          Vec3f p = prevHinv.inv() * toUnitSqInv * s;
          bbPrev.push_back( Point2f( p[0]/p[2], p[1]/p[2] ) );

          p = h * toUnitSqInv * s;
          cout << p << endl;
          bbThis.push_back( Point2f( p[0]/p[2], p[1]/p[2] ) );
        }

        for( size_t i = 0; i < 3; ++i ) {
          line( canvas, bbPrev[i], bbPrev[i+1], Scalar(255,0,0), 1 );
          line( canvas, bbThis[i], bbThis[i+1], Scalar(0,0,255), 1 );
        }
        line( canvas, bbPrev[3], bbPrev[0], Scalar(255,0,0), 1 );
        line( canvas, bbThis[3], bbThis[0], Scalar(0,0,255), 1 );
#endif


          if( first ) {
            set.addDetection( det, i );
            prevHinv = h.inv();
#ifdef DO_DRAW
        prevPts = det->points;
#endif
            first = false;
            continue;
          }

          // Draw points from unit square, transform to board space, to image space, then back again.

          const int numRand = 10000;
          int inOverlap = 0;
          for( int p = 0; p < numRand; ++p ) {
            Vec3f i( drand48(), drand48(), 1.0 );
            Vec3f o = toUnitSq * prevHinv * h * toUnitSqInv * i;
            Point2f op( o[0]/o[2], o[1]/o[2] );

            if( op.x >= 0.0 && op.x < 1.0 && op.y >= 0.0 && op.y < 1.0 ) ++inOverlap;
          }

          float overlap = inOverlap * 1.0 / numRand;

          cout << "Overlap " << overlap << endl;

#ifdef DO_DRAW
          imshow( KeyframeDebugWindowName, canvas );
          waitKey(0);
#endif


          if( overlap < _minOverlap ) {
            set.addDetection( det, i );
            prevHinv = h.inv();
#ifdef DO_DRAW
        prevPts = det->points;
#endif
          } else {
            // Only delete if not added to a set
            delete det;
          }
        }
      }

    }

  }
}


