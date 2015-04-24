
#ifndef __INTERLACED_FRAME_H__
#define __INTERLACED_FRAME_H__

#include <string>

#include <opencv2/core/core.hpp>

#include "image_pair.h"

namespace AplCam {

  using cv::Mat;
  using cv::Rect;
  using cv::Size;

  struct InterlacedFrame 
  {
    InterlacedFrame( Mat &mat )
    {
      _lines[0] = mat;
      _lines[1] = mat;

      _lines[1].data += _lines[1].step;

      for( int i = 0; i < 2; ++i ) {
        _lines[i].step[0] *= 2;
        _lines[i].size[0] /= 2;
        _lines[i].flags &= ~Mat::CONTINUOUS_FLAG;
      }

      assert( (_lines[0].rows + _lines[1].rows) == mat.rows );
    }

    //operator Mat &() { return canvas; }
    //operator cv::_InputArray() { return cv::_InputArray(canvas); }
    //operator cv::_InputOutputArray() { return cv::_InputOutputArray(canvas); }

    Mat &operator[]( int i ){ return _lines[i]; }
    const Mat &operator[]( int i ) const { return _lines[i]; }

    Size size( void ) const { return _lines[0].size(); }

    protected:

    Mat _lines[2];


  };



}

#endif

