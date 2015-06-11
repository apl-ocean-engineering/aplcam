
#ifndef __CIRCLE_BOARD_H__
#define __CIRCLE_BOARD_H__

#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "types.h"

#include "board.h"

namespace AplCam {

class CircleBoard : public Board {
 public:
  CircleBoard( const std::string &nam )
      : Board( CIRCLE, 1, 1, 0.0, nam )
  {;}

  CircleBoard( Pattern pat, const std::string &nam )
      : Board( pat, 1, 1, 0.0, nam )
  {;}

  virtual ~CircleBoard() {;}

  virtual Detection *detectPattern( const cv::Mat &gray );

 protected:

  virtual void loadCallback( cv::FileStorage &fs );

 private:
};


class ColorSegmentationCircleBoard : public CircleBoard {
 public:
  ColorSegmentationCircleBoard( const std::string &name )
      : CircleBoard( COLOR_SEG_CIRCLE, name )
  {;}

  virtual Detection *detectPattern( const cv::Mat &gray );

 protected:

  virtual void loadCallback( cv::FileStorage &fs );

};






class CircleGridBoard : public Board {
 public:
  CircleGridBoard( int w, int h, float squareSize, const std::string &nam )
      : Board( CIRCLES_GRID, w, h, squareSize, nam )
  {;}

  virtual ~CircleGridBoard() {;}

  virtual Detection *detectPattern( const cv::Mat &gray );

 protected:

  virtual void loadCallback( cv::FileStorage &fs );

 private:
};

}

#endif

