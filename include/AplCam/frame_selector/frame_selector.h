#ifndef __FRAME_SELECTOR_H__
#define __FRAME_SELECTOR_H__

#include <vector>

#include <opencv2/core/core.hpp>

namespace AplCam {

  using cv::Mat;

  class FrameSelector {
    public:
      FrameSelector( void ) {;}
      virtual ~FrameSelector( void ) {;}

      virtual bool process( Mat &img ) = 0;
  };

  class AllFrameSelector : public FrameSelector {
    public:
      AllFrameSelector( void ) {;}

      virtual bool process( Mat &img ) { return true; }
  };


  class IntervalFrameSelector : public FrameSelector {
    public:
      IntervalFrameSelector( int interval )
        : _interval(interval), _count(0) {;}

      virtual bool process( Mat &img )
      {
        if( (_count++ % _interval) == 0 ) return true;
        return false;
      }

    protected:

      int _interval, _count;

  };

  class KeyframeSelector : public FrameSelector {
    public:

      KeyframeSelector( void );

      virtual bool process( Mat &img );

    protected:

      std::vector<cv::KeyPoint> _kfKeyPoints;

  };

}

#endif
