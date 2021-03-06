#ifndef __FEATURE_TRACKER_H__
#define __FEATURE_TRACKER_H__

#include <vector>
#include <deque>
#include <list>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "AplCam/motion_model.h"

namespace AplCam {

  using cv::Mat;
  using cv::KeyPoint;
  using cv::Point2f;

  using std::vector;

  class FeatureTracker {
    public:

      struct KeyPointTrack {

        static const size_t MaxHistory = ULONG_MAX;

        KeyPointTrack( const Mat &patch, MotionModel *model );
        ~KeyPointTrack( void );

        Location predict( void  ) { return _motionModel->predict(); }
        bool search( const Mat &roi, Point2f &match );
        void update( const Mat &patch, const Point2f &position );

        Point2f pt( void ) const { return Point2f( _motionModel->pt() ); }
        Point2f vel( void ) const { return Point2f( _motionModel->vel() ); }

        std::shared_ptr<MotionModel> _motionModel;
        Mat _patch;
        std::deque< Point2f > history;

        int missed, refeatured;
      };



      FeatureTracker( void );

      void update( Mat &img, vector< KeyPoint > &kps, Mat &drawTo, float scale = 1.0 );

      void drawTracks( Mat &img, float scale = 1.0 ) { drawTracks( _tracks, img, scale ); }
      void drawTracks( const std::list<KeyPointTrack> &tracks, Mat &img, float scale = 1.0 );

      const std::list<KeyPointTrack> &tracks() const { return _tracks; }

//
//      struct TxRemoveVerticalMotion {
//        TxRemoveVerticalMotion( void )
//        {;}
//
//        bool operator()( const KeyPointTrack &pt )
//        {
//          return fabs(pt.vel().y) > 2*fabs(pt.vel().x);
//        }
//      };
//
//      void dropTracks( void )
//      {
//        std::remove_if( _tracks.begin(), _tracks.end(), TxRemoveVerticalMotion() );
//      }


    protected:

      Mat patchROI( Mat &img, const Point2f &center )
      {
        return Mat( img, cv::Rect( center.x - _patchRadius, center.y - _patchRadius,
              2*_patchRadius + 1, 2*_patchRadius+1 ) );
      }

      bool tooNearEdge( const Mat &img, const Point2f &pt )
      {
        return ( pt.x - _patchRadius < 0 || pt.y - _patchRadius < 0 ||
            pt.x + _patchRadius >= img.size().width ||
            pt.y + _patchRadius >= img.size().height );
      }




      Mat  _previous;
      std::list< KeyPointTrack > _tracks;

      static const float _dropRadius;
      static const float _patchRadius;
      static const int _maxMisses;
      static const int _maxTracks;
  };

}


#endif
