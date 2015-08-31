#ifndef __HISTOGRAM_H__
#define __HISTOGRAM_H__

#include <vector>

#include <opencv2/core.hpp>

namespace AplCam {

  using std::vector;

  class Gaussian {
  public:
    Gaussian( double mean, double var )
    : _mean(mean), _var(var)
    {;}

    double mean( void ) const { return _mean; }
    double var( void )  const { return _var; }
    double stddev( void ) const { return sqrt(_var); }
    double sigma( void )  const { return stddev(); }
  protected:

    double _mean, _var;
  };

  template <typename _Tp>
  class _Histogram {
  public:
    _Histogram( unsigned int numBins ); //, float min = 0.0, float max = 1.0 );

    unsigned int numBins( void ) const { return _bins.size(); }

    void add( float value );

    vector<float> percentages( void ) const;

    template <typename _St = _Tp>  _St sum( void ) const;

    cv::Mat draw( unsigned int height, float scale = 1.0 ) const;
    cv::Mat draw( unsigned int height, const Gaussian &g, float scale = 1.0, float sigma = 1.0 ) const;

    Gaussian fitGaussian( void ) const;

  protected:

    vector<_Tp> _bins;

    //float _min, _max;
  };

  typedef _Histogram<unsigned int> Histogram;

}


#include "histogram_impl.h"

#endif
