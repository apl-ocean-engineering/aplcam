
#ifndef __DISTORTION_RADIAL_POLYNOMIAL_H__
#define __DISTORTION_RADIAL_POLYNOMIAL_H__

#include <opencv2/core.hpp>

#include "distortion_model.h"

namespace cv {
  typedef Vec<double,5> Vec5d;
  typedef Vec<double,8> Vec8d;
}

namespace Distortion {

  using cv::Vec4d;
  using cv::Vec5d;
  using cv::Vec8d;

  class OpencvRadialPolynomial : public DistortionModel {
    public:


      OpencvRadialPolynomial( void );
      OpencvRadialPolynomial( const Vec8d &distCoeffs );
      OpencvRadialPolynomial( const Vec5d &distCoeffs );
      OpencvRadialPolynomial( const Vec4d &distCoeffs );
      OpencvRadialPolynomial( const Vec8d &distCoeffs, const Matx33d &cam );
      OpencvRadialPolynomial( const Vec5d &distCoeffs, const Matx33d &cam );
      OpencvRadialPolynomial( const Vec4d &distCoeffs, const Matx33d &cam );

      //void set(const cv::Vec2d& f, const cv::Vec2d& c, const double &alpha = 0, const cv::Vec4d& k = Vec4d(0,0,0,0) )
      //{
      //  setCamera( f[0], f[1], c[0], c[1], alpha );
      //  _distCoeffs = k;
      //}

      static const std::string Name( void ) { return "OpencvRadialPolynomial"; }
      virtual const std::string name( void ) const { return OpencvRadialPolynomial::Name(); }

      Vec8d distCoeffs( void ) const    { return _distCoeffs; }

      virtual cv::Mat distortionCoeffs( void ) const 
      { 
        cv::Mat m = (cv::Mat_<double>(8,1) << _distCoeffs[0], _distCoeffs[1], _distCoeffs[2], _distCoeffs[3],
            _distCoeffs[4], _distCoeffs[5], _distCoeffs[6], _distCoeffs[7]);
        return m;
      }


      //static RadialPolynomial Calibrate( const ObjectPointsVecVec &objectPoints, 
      //    const ImagePointsVecVec &imagePoints, const Size& image_size,
      //    vector< Vec3d > &rvecs, 
      //    vector< Vec3d > &tvecs,
      //    int flags = 0, 
      //    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

      virtual void projectPoints( const ObjectPointsVec &objectPoints, 
          const Vec3d &_rvec, const Vec3d &_tvec, ImagePointsVec &imagePoints ) const;

      virtual cv::FileStorage &write( cv::FileStorage &out ) const;
      static OpencvRadialPolynomial *Load( cv::FileStorage &in );

    protected: 

      virtual bool doCalibrate( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          CalibrationResult &result,
          int flags = 0, 
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

      virtual ImagePoint undistort( const ImagePoint &pw ) const;
      virtual ImagePoint distort( const Vec3f &w ) const ;

      static const Vec8d InitialDistortionEstimate( void ) 
      { return Vec8d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

      cv::Vec8d _distCoeffs;

  };

  class CeresRadialPolynomial : public OpencvRadialPolynomial {
    public:

      CeresRadialPolynomial( void );
      CeresRadialPolynomial( const Vec8d &distCoeffs );
      CeresRadialPolynomial( const Vec5d &distCoeffs );
      CeresRadialPolynomial( const Vec4d &distCoeffs );
      CeresRadialPolynomial( const Vec8d &distCoeffs, const Matx33d &cam );
      CeresRadialPolynomial( const Vec5d &distCoeffs, const Matx33d &cam );
      CeresRadialPolynomial( const Vec4d &distCoeffs, const Matx33d &cam );

      void set( const double *c, const double alpha )
      {
        setCamera( c[0], c[1], c[2], c[3], alpha );
      }

      static const std::string Name( void ) { return "CeresRadialPolynomial"; }
      virtual const std::string name( void ) const { return CeresRadialPolynomial::Name(); }

      //virtual cv::FileStorage &write( cv::FileStorage &out ) const;
      static CeresRadialPolynomial *Load( cv::FileStorage &in );

    protected: 

      virtual bool doCalibrate( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          CalibrationResult &result,
          int flags = 0, 
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

  };




}


#endif
