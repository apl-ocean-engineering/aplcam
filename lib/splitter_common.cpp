
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

#include "splitter_common.h"

using namespace std;
using namespace cv;

namespace AplCam {

  bool SplitterOpts::parseArgs( int argc, char **argv, stringstream &msg ) 
  {

    try{
      TCLAP::CmdLine cmd("Video splitter", ' ', "0.1");

      doParse( cmd, argc, argv );
    } 
    catch (TCLAP::ArgException &e)  // catch any exceptions
    { 
      msg << "error: " << e.error() << " for arg " << e.argId() << std::endl;
      return false;
    }


    return validate( msg );
  }

  void SplitterOpts::doParse( TCLAP::CmdLine &cmd, int argc, char **argv )
  { 
    TCLAP::SwitchArg displayArg("D", "display", "Show progress in window", cmd, false );
    TCLAP::ValueArg< string > outputDirArg("o", "output-dir", "Directory for resulting images", 
        false, ".", "dir", cmd );
    TCLAP::ValueArg< int > waitKeyArg("w", "wait-key", "Arg for waitkey()", 
        false, 0, "ms", cmd );
    TCLAP::ValueArg< float > seekToArg("s", "seek-to", "For video sources, seek <sec> frames into the video before starting", false, -1, "sec", cmd );

    TCLAP::ValueArg< float > scaleDisplayArg("S", "scale-display", "If displaying, scale to <mp> megapixels", false, -1, "megapix", cmd );

    TCLAP::UnlabeledMultiArg< string > imgNamesArg("image_files", "Image files for processing", true, "file names", cmd );

    cmd.parse( argc, argv );

    imgNames = imgNamesArg.getValue();
    doDisplay = displayArg.getValue();
    outputDir = outputDirArg.getValue();
    seekTo = seekToArg.getValue();
    scaleDisplay = scaleDisplayArg.getValue();
  }



  bool SplitterOpts::validate( stringstream &msg )
  {
    if( !directory_exists( outputDir ) ) {
      msg << "Directory \"" << outputDir << "\" doesn't exist.";
      return false;
    }

    return true;
  }








  SplitterApp::SplitterApp( SplitterOpts options )
    : _selector( options.makeSelector() ), _splitterOpts( options )
  {;}


  SplitterApp::SplitterApp( SplitterOpts options, FrameSelector *selector = NULL )
    : _selector( selector ), _splitterOpts( options )
  {;}


  SplitterApp::~SplitterApp( void )
  {
    if( _selector != NULL ) delete _selector;
  }



  bool SplitterApp::run( void ) {
    if( _splitterOpts.doDisplay ) namedWindow( winname );
    int _waitKey = _splitterOpts.waitKey;

    FrameSource *source = NULL;
    if( _splitterOpts.imgNames.size() == 1 ) {
      // Assume it's a video
      LOG(INFO) << "Attepting to open video " << _splitterOpts.imgNames[0];
      VideoSource *vid = new VideoSource( _splitterOpts.imgNames[0] );

      if( _splitterOpts.seekTo > 0 ) vid->seekToSeconds( _splitterOpts.seekTo );
      if( _waitKey < 0 ) _waitKey = round(1000 * 1/vid->fps() );

      source = vid;
    } else {
      LOG(INFO) << "Loading list of " << _splitterOpts.imgNames.size() << " images";
      source = new ListOfImages( _splitterOpts.imgNames );
      if( _waitKey < 0 ) _waitKey = 1;
    }

    if( source == NULL && source->isOpened() ) {
      LOG(ERROR) << "Couldn't create a frame source";
      return false;
    }

    if( _selector == NULL ) LOG(WARNING) << "No frame selector specified.";


    Mat img, toDisplay;
    bool done = false;
    _frame = 0;
    int wk = _waitKey;
    while( source->read( img )  && !done) {

      toDisplay.release();

      if( _selector ) {
        if( _selector->process( img ) ) {
          toDisplay = img;
          processSplitFrame( img, toDisplay );
        } else {
          processRejectedFrame( img, toDisplay );
        }
      }

      if( _splitterOpts.doDisplay && !toDisplay.empty() ) {

        if( _splitterOpts.scaleDisplay > 0 ) {
          // Shouldn't do this calculation every time
          float factor = _splitterOpts.scaleDisplay  * 1000000 / toDisplay.size().area();

          Mat out;
          resize( toDisplay, out, Size(), factor, factor );
          imshow( winname, out );
        } else {
          imshow( winname, toDisplay );
        }

        char c = waitKey( wk );
        if( c == 'q' ) done=true;
        else if ( c == ' ' ) {
          if( wk == 0 ) wk = _waitKey;
          else wk = 0;
        }
      }

      _frame++;
    }


    return true;
  }


}
