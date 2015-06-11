
#ifndef __IMAGE_ACCUMULATOR_H__
#define __IMAGE_ACCUMULATOR_H__

#include <vector>
#include <opencv2/core.hpp>

namespace AplCam {

using std::vector;
using cv::Mat;

class ImageAccumulator {
 public:

  ImageAccumulator();

  bool addImage( const Mat &mat );

  Mat mean( void );
  Mat var( void );

 protected:

  void update( void );

  vector< Mat > _imgs;

  bool _upToDate;
  Mat _mean, _var;

};

}

#endif
