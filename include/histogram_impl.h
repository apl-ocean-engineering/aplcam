
#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace AplCam {

  template <typename _Tp>
  _Histogram<_Tp>::_Histogram( unsigned int numBins ) //, float mn, float mx )
  : _bins( numBins )  //,  _min( std::min( mn, mx) ), _max( std::max(mn,mx) )
  {
    ;
  }

  template <typename _Tp>
  void _Histogram<_Tp>::add( float bin )
  {
    //unsigned int bin = floor( (value-_min) / (_max-_min) * numBins() );

    if( (bin > 0) && (bin < numBins()) ) ++_bins[bin];
  }

  template <typename _Tp>
  vector<float> _Histogram<_Tp>::percentages( void ) const
  {
    vector<float> pct( numBins(), 0.0 );

    float total = 1.f / sum<float>();

    for( unsigned int i = 0; i < numBins(); ++i )
      pct[i] = _bins[i] * total;

    return pct;
  }

  template <typename _Tp>
  template <typename _St>
  _St _Histogram<_Tp>::sum( void ) const
  {
    _St t = 0;
    for( unsigned int i = 0; i < numBins(); ++i ) t += _bins[i];
    return t;
  }

  template <typename _Tp>
  cv::Mat _Histogram<_Tp>::draw( unsigned int height, float scale ) const
  {
    cv::Mat img( cv::Size(numBins(), height), CV_8UC3, cv::Scalar(0,0,0) );
    vector<float> pct( percentages() );

    for( cv::Point p(0,0); p.x < numBins(); ++p.x ) {
      for( p.y = height*(1.0-pct[p.x]); p.y < height; ++p.y ){
        img.at<cv::Vec3b>(p) = cv::Vec3b(255,255,255);
      }
    }

    Mat scaled;
    if( scale == 1.0 )  scaled = img;
    else                cv::resize( img, scaled, Size(), scale, scale, cv::INTER_NEAREST );


    return scaled;
  }

  template <typename _Tp>
  cv::Mat _Histogram<_Tp>::draw( unsigned int height,  const Gaussian &gaussian, float scale, float sigma ) const
  {
    Mat img( draw( height, scale ) );

    float mean = gaussian.mean(), stddev = gaussian.sigma(), ss = stddev*sigma;

    {
      double at = mean;
      if( (at >= 0) && (at < numBins()) )
      for( cv::Point p(at, 0); p.y < height; ++p.y ) img.at<cv::Vec3b>(p*scale) = cv::Vec3b(0,0,255);
    }

    {
      double at = mean + stddev*sigma;
      if( (at >= 0) && (at < numBins()) )
      for( cv::Point p(at, 0); p.y < height; ++p.y ) img.at<cv::Vec3b>(p*scale) = cv::Vec3b(255,0,255);
    }

    {
      double at = mean - stddev*sigma;
      if( (at >= 0) && (at < numBins()) )
      for( cv::Point p(at, 0); p.y < height; ++p.y ) img.at<cv::Vec3b>(p*scale) = cv::Vec3b(255,0,255);
    }
    return img;

  }

  template <typename _Tp>
  Gaussian _Histogram<_Tp>::fitGaussian( void ) const
  {
    // Fit a simple gaussian to the cumulative data
    double mean = 0.0, var = 0.0;

    float total = 1.f / sum<float>();
    for( unsigned int x = 0; x < numBins(); ++x ) mean += x * _bins[x] * total;
    for( unsigned int x = 0; x < numBins(); ++x ) var += _bins[x] * total * powf(x - mean, 2);

    return Gaussian( mean, var );
  }

}
