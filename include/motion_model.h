
#ifndef __MOTION_MODEL_H__
#define __MOTION_MODEL_H__

#include <vector>
#include <deque>
#include <list>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace AplCam {

  using cv::Mat;
  using cv::KeyPoint;
  using cv::Point2f;


  struct Location {
    Location( const Point2f &_pt, const Point2f &_cov ) 
      : pt( _pt ), cov( _cov ) {;}
    Location( float x, float y, float covx, float covy )
      : pt( x,y ), cov( covx, covy ) {;}

    cv::Point2f pt;
    cv::Point2f cov;
  };


  struct MotionModel {
    MotionModel( void ) {;};
    virtual ~MotionModel() {;}

    virtual Location predict( void ) = 0;
    virtual void update( const Point2f &loc ) = 0;

    virtual Point2f pt( void ) const = 0;
  };

  struct DecayingVelocityMotionModel : public MotionModel {
    DecayingVelocityMotionModel( const Point2f &x_0 );
    virtual ~DecayingVelocityMotionModel() {;}

    virtual Location predict( void );
    virtual void update( const Point2f &loc );

    virtual Point2f pt( void ) const { return Point2f( _state(0), _state(1) ); }

    float alpha;
    cv::Vec4f _state;
    cv::Matx44f _cov;
  };

};


#endif
