#include "motion_model.h"


namespace AplCam {

  using namespace cv;

  typedef Matx<float,2,4> Matx24f;
  typedef Matx<float,4,2> Matx42f;
  typedef Matx<float,2,1> Matx21f;



  //===========================================================================
  //
  // Basic 2dof Kalman Filter
  //
  // Model is:
  //
  // x_t+1 = x_t + v_t dt
  // v_t+1 = alpha v_t 
  //
  //


  DecayingVelocityMotionModel::DecayingVelocityMotionModel( const Point2f &x_0 )
    : MotionModel(), alpha(0.7),_state( x_0.x, x_0.y, 0, 0 ), _cov( Matx44f::eye() )
  {
    _cov(0,0) = _cov(1,1) = 5;
    _cov(2,2) = _cov(3,3) = 25;
  }

  Location DecayingVelocityMotionModel::predict( void  )
  {
    // Assume fixed rate operation
    //   Do the matrix-based approach for now.  Could just expand out the equations
    //   later for efficiency (?)
    //
    // State change matrix
    const Matx44f F( 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, alpha, 0,
        0, 0, 0, alpha );
    // Process noise
    const Matx44f Q( 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 25, 0,
        0, 0, 0, 25 );

    _state = F * _state;
    _cov = F * _cov * F.t() + Q;

    return Location( _state(0), _state(1), _cov(0,0), _cov(1,1) );
  }

  void DecayingVelocityMotionModel::update( const Point2f &l )
  {
    // Measurement matrix
    const Matx24f H( 1, 0, 0, 0,
        0, 1, 0, 0 );

    // Observation covariances
    Matx21f y = Matx21f( l.x, l.y ) - H*_state;
    Matx22f R = 2 * Matx22f::eye();

    Matx22f S = H * _cov * H.t() + R;
    Matx42f K = _cov * H.t() * S.inv();

    _state += K * y;
    _cov = (Matx44f::eye() - K*H)*_cov;

  }

}
