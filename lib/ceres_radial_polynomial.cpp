
#include <math.h>

#include "distortion_radial_polynomial.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <boost/thread.hpp>

#include <iostream>
using namespace std;

namespace Distortion {

  using namespace cv;
  using namespace std;

  CeresRadialPolynomial::CeresRadialPolynomial( void )
    : RadialPolynomial()
  {;}

  CeresRadialPolynomial::CeresRadialPolynomial( const Vec4d &d )
    : RadialPolynomial( d )
  {;}

  CeresRadialPolynomial::CeresRadialPolynomial( const Vec5d &d )
    : RadialPolynomial( d )
  {;}

  CeresRadialPolynomial::CeresRadialPolynomial( const Vec8d &d )
    : RadialPolynomial( d )
  {;}

  CeresRadialPolynomial::CeresRadialPolynomial( const Vec4d &d, const Matx33d &cam )
    : RadialPolynomial( d, cam )
  {;}

  CeresRadialPolynomial::CeresRadialPolynomial( const Vec5d &d, const Matx33d &cam )
    : RadialPolynomial( d, cam )
  {;}

  CeresRadialPolynomial::CeresRadialPolynomial( const Vec8d &d, const Matx33d &cam )
    : RadialPolynomial( d, cam )
  {;}


  // Ceres functor for solving radial calibration problem
  //  Based on the Bundler solver used in their examples
  struct RadialDistortionReprojError {
    RadialDistortionReprojError(double obs_x, double obs_y, double world_x, double world_y )
      : observedX(obs_x), observedY(obs_y), worldX( world_x ), worldY( world_y ) {}

    template <typename T>
      bool operator()(const T* const camera,
          const T* const alpha,
          const T* const k12,
          const T* const p12,
          const T* const k3,
          const T* const k456,
          const T* const pose, 
          T* residuals) const
      {
        //
        // camera is a 4-vector
        //    2 focal length
        //    2 camera center
        //
        // alpha is a 1-vector (separate so it can be set Constant/Variable)
        //
        // k12 is a 2-vector: k1 and k2 by opencv
        // p12 is a 2-vector: p1 and p2 by opencv
        // k3  is a 1-vector: k3 by openv
        // k456 is a 3-vector: k456 by opencv
        //
        // pose is a 6-vector
        //    3 angles
        //    3 translations
        // pose[0,1,2] are an angle-axis rotation.
        //
        T point[3] = { T( worldX ), T( worldY ), T( 0.0 ) };
        T p[3];
        ceres::AngleAxisRotatePoint(pose, point, p);
        p[0] += pose[3]; 
        p[1] += pose[4]; 
        p[2] += pose[5];

        const T &fx = camera[0];
        const T &fy = camera[1];
        const T &cx = camera[2];
        const T &cy = camera[3];
        const T &k1 = k12[0], &k2 = k12[1];
        const T &p1 = p12[0], &p2 = p12[1];
        const T &k4 = k456[0], &k5 = k456[1], &k6 = k456[2];

        T xp = p[0]/p[2], yp = p[1]/p[2];
        T r2 = xp*xp + yp*yp;
        T r4 = r2*r2;
        T r6 = r2*r4;

        T xpp = xp * ( T(1) + k1*r2 + k2*r4 + k3[0]*r6 ) / ( T(1) + k4*r2 + k5*r4 + k6*r6 ) + T(2)*p1*xp*yp + p2*(r2 + T(2)*xp*xp);
        T ypp = yp * ( T(1) + k1*r2 + k2*r4 + k3[0]*r6 ) / ( T(1) + k4*r2 + k5*r4 + k6*r6 ) + p1*(r2 + T(2)*yp*yp) + T(2)*p2*xp*yp;;

        T predictedX = fx*(xpp + alpha[0]*ypp) + cx;
        T predictedY = fy* ypp                 + cy;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predictedX - T(observedX);
        residuals[1] = predictedY - T(observedY);
        return true;
      }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x, const double observed_y, 
        const double world_x, const double world_y ) {
      return (new ceres::AutoDiffCostFunction<RadialDistortionReprojError, 2, 4,1,2,2,1,3, 6>(
            new RadialDistortionReprojError(observed_x, observed_y, world_x, world_y)));
    }

    double observedX, observedY;
    double worldX, worldY;
  };

  bool CeresRadialPolynomial::doCalibrate(
      const ObjectPointsVecVec &objectPoints, 
      const ImagePointsVecVec &imagePoints, 
      const Size& image_size,
      CalibrationResult &result,
      int flags, 
      cv::TermCriteria criteria)
  {

    // Check and see if the camera matrix has been initialized
    if( norm( matx(), Mat::eye(3,3,CV_64F) ) < 1e-9 )
      setCamera( Camera::InitialCameraEstimate( image_size ) );

    int totalPoints = 0;
    int goodImages = 0;

    for( size_t i = 0; i < objectPoints.size(); ++i )  {
      if( result.status[i] ) {
        ImagePointsVec undistorted =  unwarp( normalize( imagePoints[i] ) );

        // Found the approach provided by initExtrinsics to be more reliable (!)
        // will need to investigate why that is.
        bool pnpRes = solvePnP( objectPoints[i], undistorted, Mat::eye(3,3,CV_64F), Mat(), 
            result.rvecs[i], result.tvecs[i], false, CV_ITERATIVE );

        //cout << "Pnp: " << (pnpRes ? "" : "FAIL") << endl << result.rvecs[i] << endl << result.tvecs[i] << endl;
        //initExtrinsics( imagePoints[i], objectPoints[i], rvecs[i], tvecs[i] );
        //cout << "initExtrinsics: " << endl << rvecs[i] << endl << tvecs[i] << endl;

        if( !pnpRes ) { 
          result.status[i] = false; 
          continue; 
        }

        ++goodImages;
        totalPoints += objectPoints[i].size();
      }
    }

    cout << "From " << objectPoints.size() << " images, using " << totalPoints << " from " << goodImages << " images" << endl;

    double camera[9] = { _fx, _fy, _cx, _cy };
    double alpha = _alpha;

    // Inherently fragile.  Store coeffs in a double array instead?
    double k12[2] = { _distCoeffs[0], _distCoeffs[1] },
           p12[2] = { _distCoeffs[2], _distCoeffs[3] },
           k3[1]  = { _distCoeffs[4] },
           k456[3] = { _distCoeffs[5], _distCoeffs[6], _distCoeffs[7] };

    if( ! (flags & CV_CALIB_RATIONAL_MODEL ) ) {
      k456[0] = 0.0;
      k456[1] = 0.0;
      k456[2] = 0.0;
    }

    if( flags & CV_CALIB_ZERO_TANGENT_DIST ) {
      p12[0] = 0.0;
      p12[1] = 0.0;
    }

    double *pose = new double[ goodImages * 6];

    ceres::Problem problem;
    for( size_t i = 0, idx = 0; i < objectPoints.size(); ++i ) {
      if( result.status[i] ) {

        double *p = &( pose[idx*6] );

        // Mildly awkward
        p[0] = result.rvecs[i][0];
        p[1] = result.rvecs[i][1];
        p[2] = result.rvecs[i][2];
        p[3] = result.tvecs[i][0];
        p[4] = result.tvecs[i][1];
        p[5] = result.tvecs[i][2];

        ++idx;

        for( size_t j = 0; j < imagePoints[i].size(); ++j ) {
          ceres::CostFunction *costFunction = RadialDistortionReprojError::Create( imagePoints[i][j][0], imagePoints[i][j][1],
                                                                              objectPoints[i][j][0], objectPoints[i][j][1] );
          problem.AddResidualBlock( costFunction, NULL, camera, &alpha, k12, p12, k3, k456, p );
        }
      }
    }

    if( flags & CALIB_FIX_SKEW ) problem.SetParameterBlockConstant( &alpha );
    if( flags & CV_CALIB_ZERO_TANGENT_DIST ) problem.SetParameterBlockConstant( p12 );
    if( ! (flags & CV_CALIB_RATIONAL_MODEL ) ) problem.SetParameterBlockConstant( k456 );

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = criteria.maxCount;
    options.minimizer_progress_to_stdout = true;

    // This should be configurable by the end user
    options.num_threads = boost::thread::hardware_concurrency();

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    for( size_t i = 0, idx=0; i < objectPoints.size(); ++i ) {
      if( result.status[i] ) {
        result.rvecs[i] = Vec3d( &(pose[idx*6]) );
        result.tvecs[i] = Vec3d( &(pose[idx*6+3]) );
        ++idx;
      }
    }

    delete[] pose;

    result.totalTime = summary.total_time_in_seconds;
    
    // N.b. the Ceres cost is 1/2 || f(x) ||^2
    //
    // Whereas we usually use the RMS reprojection error
    //
    // sqrt( 1/N  ||f(x)||^2
    //
    // So rms = sqrt( 2/N final_cost )
    //
    result.residual = summary.final_cost;
    result.numPoints = totalPoints;
    result.numImages = goodImages;
    result.good = summary.IsSolutionUsable();

    set(camera, alpha);

    // Ugly and awkward
    _distCoeffs[0] = k12[0];
    _distCoeffs[1] = k12[1];
    _distCoeffs[2] = p12[0];
    _distCoeffs[3] = p12[1];
    _distCoeffs[4] = k3[0];
    _distCoeffs[5] = k456[0];
    _distCoeffs[6] = k456[1];
    _distCoeffs[7] = k456[2];

    result.rms = reprojectionError( objectPoints, result.rvecs, result.tvecs, imagePoints, result.reprojErrors, result.status );

    cout << "Final camera: " << endl << matx() << endl;
    cout << "Final distortions: " << endl << _distCoeffs << endl;

    return true;
  }

//  bool CeresRadialPolynomial::doCalibrate(
//      const ObjectPointsVecVec &objectPoints, 
//      const ImagePointsVecVec &imagePoints, 
//      const Size& imageSize,
//      CalibrationResult &result,
//      int flags, 
//      cv::TermCriteria criteria)
//  {
//
//    Mat camera( mat() );
//    Mat dist( _distCoeffs );
//
//    vector<Mat> _rvecs, _tvecs;
//
//    ObjectPointsVecVec _objPts;
//    ImagePointsVecVec _imgPts;
//
//    for( size_t i = 0; i < objectPoints.size(); ++i ) {
//      if( result.status[i] ) {
//        _objPts.push_back( objectPoints[i] );
//        _imgPts.push_back( imagePoints[i] );
//      }
//    }
//
//    //for( int i = 0; i < objectPoints.size(); ++i ) 
//    //  cout << i << " " << objectPoints[i].size() << " " << imagePoints[i].size() << endl;
//
//    int before = getTickCount();
//    result.rms = calibrateCamera( _objPts, _imgPts, imageSize, camera, dist, _rvecs, _tvecs, flags, criteria );
//    result.totalTime = (getTickCount() - before)/getTickFrequency();
//
//
//    setCamera( camera );
//
//    // _distCoeffs will be variable length
//    double *d = dist.ptr<double>(0);
//
//    for( unsigned int i = 0; i < 4; ++i ) _distCoeffs[i] = d[i];
//    if( (dist.rows*dist.cols) == 5 )  _distCoeffs[4] = d[4];
//    if( (dist.rows*dist.cols) == 8 ) {
//      _distCoeffs[5] = d[5];
//      _distCoeffs[6] = d[6];
//      _distCoeffs[7] = d[7];
//    }
//
//    for( size_t i =0, k = 0; i < objectPoints.size(); ++i ) {
//      if( result.status[i] ) {
//        result.rvecs[i] = _rvecs[k];
//        result.tvecs[i] = _tvecs[k];
//        ++k;
//      }
//    }
//
//    result.good = true;
//    return result.good;
//  }

  CeresRadialPolynomial *CeresRadialPolynomial::Load( cv::FileStorage &in )
  {
    Mat kmat, distmat;

    in["camera_matrix"] >> kmat;
    in["distortion_coefficients"] >> distmat;

    Matx33d k;
    Vec8d dist;

    kmat.copyTo( k, CV_64F );
    distmat.copyTo( dist, CV_64F );

    return new CeresRadialPolynomial( dist, k );

  }
}


