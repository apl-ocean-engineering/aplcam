
#ifndef __IMAGE_ACCUMULATOR_H__
#define __IMAGE_ACCUMULATOR_H__

#include <vector>
#include <opencv2/core/core.hpp>

namespace AplCam {

using std::vector;
using cv::Mat;

class ImageAccumulator {
 public:

  ImageAccumulator();

  bool add( const Mat &mat );

  Mat mean( void );
  Mat var( void );

  size_t size( void ) const { return _imgs.size(); }

 protected:

  void update( void );

  vector< Mat > _imgs;

  bool _upToDate;
  Mat _mean, _var;

};

}

#endif
