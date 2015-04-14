#ifndef __FRAME_SOURCE_H__
#define __FRAME_SOURCE_H__

#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::vector;
using std::string;
using cv::Mat;

namespace AplCam {

  class FrameSource {
    public:
      FrameSource() {;}

      virtual bool isOpened( void ) const { return true; };
      virtual bool read( Mat & ) = 0;


  };

  class ListOfImages : public FrameSource {
    public:

      ListOfImages( const vector< string > &imgFiles )
        : _imgFiles( imgFiles ), _idx(0)
      {;}

      virtual bool read( Mat &img )
      {
        if( _idx < _imgFiles.size() ) {
          img = cv::imread( _imgFiles[ _idx++ ] );
          return true;
        }

        return false;
      }

    protected:

      vector< string > _imgFiles;
      size_t _idx;

  };


  class VideoSource : public FrameSource {
    public:

      VideoSource( const string &vidFile )
        : _vidFile( vidFile ), _vid( _vidFile )
      {;}

      virtual bool isOpened( void ) const 
      { 
        return _vid.isOpened(); 
      }

      virtual bool read( Mat &img )
      {
        return _vid.read( img );
      }

      bool seekToSeconds( float sec )
      {
        return _vid.set( CV_CAP_PROP_POS_MSEC, sec * 1000.f );
      }

      float fps( void )
      {
        return _vid.get( CV_CAP_PROP_FPS );
      }

    protected:

      string _vidFile;
      cv::VideoCapture _vid;

  };

}

#endif
