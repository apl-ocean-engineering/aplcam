
#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "libg3logger/g3logger.h"
#include "AplCam/board/trailer_hitch.h"
#include "AplCam/detection/circle.h"

namespace AplCam {
  using namespace std;
  using namespace cv;

  //===========================================================================
  //  TrailerHitch
  //===========================================================================

  TrailerHitch::TrailerHitch( const std::string &name )
    : ColorSegmentationCircleBoard(name)
  {;}

  TrailerHitch::~TrailerHitch()
  {;}

   void TrailerHitch::to_json( json &j ) const {
     ;
   }

   void TrailerHitch::from_json( const json &j ) {
     _targets.clear();

     const json &targets( j["targets"] );
     for (json::const_iterator it = targets.begin(); it != targets.end(); ++it) {

       const json &params( it.value() );
       if( params.count("hue") > 0 ) {
         HueTarget t( it.key(), params["hue"] );
         _targets.push_back(t);
       } else {
         LOG(WARNING) << "Couldn't figure out how to make the target \"" << it.key();
       }
     }
   }


  Detection *TrailerHitch::detectPattern( const cv::Mat &img )
  {

    LOG(INFO) << "Detect pattern!!";

    for( auto const &target : _targets ) {
      auto det = target.detectPattern( img );
    }

    return nullptr;
  }

  Detection *TrailerHitch::HueTarget::detectPattern( const cv::Mat &img ) const
  {

    // Desired color in BGR
    // Scalar targetColor( 0, 128, 255 );
    // Vec3f  targetColorF( targetColor[0]/256, targetColor[1]/256, targetColor[2]/256 );

    const string hWindow = "channel0 - H",
          sWindow = "channel1 - S",
          vWindow = "channel2 - V";
    const string diffWindow = "diff";
    const string enhancedWindow = "enhanced";

    // Compute hue value for target color
    // float B = targetColorF[0], G = targetColorF[1], R = targetColorF[2];
    // float alpha = 0.5 * ( 2* R - B - G );
    // float beta = 0.86603 * ( G - B );    // Constant is sqrt(3)/2
    // float targetAng = atan2( beta, alpha );

    //Scalar targetCS( cos( ang ), sin( ang ) );

    Mat imgF, hsv;
    img.convertTo( imgF, CV_32FC3, 1.0/255 );
    cvtColor( imgF, hsv, CV_BGR2HSV );

    vector<Mat> channels;
    split( hsv, channels );

    //  imshow( hWindow, channels[0] / 360.0 );
    //  imshow( sWindow, channels[1] );
    //  imshow( vWindow, channels[2] );


    Mat diff( channels[0].size(), CV_32FC1 );
    assert( diff.type() == CV_32FC1 &&
        channels[0].type() == CV_32FC1 &&
        channels[1].type() == CV_32FC1 &&
        channels[2].type() == CV_32FC1 );

    float *d = (float *)diff.data,
          *h = (float *)channels[0].data,
          *s = (float *)channels[1].data,
          *v = (float *)channels[2].data;

    for( int i = 0; i < (diff.rows * diff.cols); i++ )  {
      d[i] = (cos( h[i] - _hue ) + 1) * 0.5;
      d[i] *= s[i] * v[i];
      d[i] = powf( d[i], 2 );
    }

    // Normalize diff so max(diff) = 1
    double mn, mx;
    minMaxLoc( diff, &mn, &mx );
    diff *= 1.0/mx;

    imwrite( "/tmp/diffHue.jpg", diff );

    Mat diffBlur;
    dilate( diff, diff, Mat() );
    GaussianBlur( diff, diffBlur, Size(7,7), 3, 3  );

    // V = diff
    // S = 1 - diff
    diffBlur.copyTo( channels[2] );
    subtract( Scalar(1), diffBlur, channels[1] );

  //  Mat enhanced, enhancedBGR;
  //  merge( channels, enhanced );
  //  cvtColor( enhanced, enhancedBGR, CV_HSV2BGR );
  //
  //  imshow(enhancedWindow, enhancedBGR );

    Mat gray;
    diff.convertTo( gray, CV_8UC1, 255 );

    const string grayWindow = "gray";
    imwrite( "/tmp/diffGray.jpg", gray );

    Mat blurred;
    GaussianBlur( gray, blurred, Size(9,9), 2,2 );

    Mat canny;
    cv::Canny( blurred, canny, 50, 100 );
    const string cannyWindow = "canny";
    canny *= 255;
    imwrite( "/tmp/diffCanny.jpg", canny );


    vector<Vec3f> circles;
    const float accumRes = 1, minDist = 4;
    cv::HoughCircles( blurred, circles, CV_HOUGH_GRADIENT, accumRes, minDist, 100, 25 );

    return new CircleDetection( circles );
  }


}
