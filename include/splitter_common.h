#ifndef __SPLITTER_COMMON_H__
#define __SPLITTER_COMMON_H__


#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tclap/CmdLine.h>
#include <glog/logging.h>

#include "file_utils.h"

#include "frame_source.h"

#include "frame_selector/frame_selector.h"

using namespace std;
using namespace cv;

namespace AplCam {

  struct SplitterOpts {
    public:
      SplitterOpts( )
        : waitKey(-1), 
        seekTo(-1), 
        scaleDisplay( -1 ),
        doDisplay( false ), 
        outputDir()
    {;}

      //typedef enum {EXTRACT_SINGLE, EXTRACT_INTERVAL,  NONE = -1} Verbs;

      int waitKey;
      float  seekTo, scaleDisplay;
      bool doDisplay;
      vector< string > imgNames;
      string outputDir;

      bool parseArgs( int argc, char **argv, stringstream &msg );
      virtual void doParse( TCLAP::CmdLine &cmd, int argc, char **argv );

      virtual bool validate( stringstream &msg );

      FrameSelector *makeSelector( void )
      { return NULL; }
  };



  class SplitterApp
  {
    public:
      SplitterApp( SplitterOpts options );
      SplitterApp( SplitterOpts options, FrameSelector *selector );

      virtual ~SplitterApp( void );

      virtual bool run( void );

      virtual bool processSplitFrame( Mat &img, Mat &toDisplay ) 
      { toDisplay = img; return true; }

      virtual bool processRejectedFrame( Mat &img, Mat &toDisplay )
      { return true; }

    protected:

      FrameSelector *_selector;

      int _frame;

    private:
      SplitterOpts _splitterOpts;

      const string winname = "input";
  };


}

#endif
