
#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tclap/CmdLine.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;

struct ExtractOpts {
 public:
  ExtractOpts()
      : seekTo(-1)
  {;}


  int seekTo;
  vector< string > inFiles;

  bool parseOpts( int argc, char **argv, stringstream &msg )
  {

    try {
      TCLAP::CmdLine cmd("extract a single frame from files", ' ', "0.1" );

      TCLAP::UnlabeledMultiArg< std::string > fileNamesArg("files", "Files", "true", "file names", cmd );

      cmd.parse( argc, argv );

      inFiles = fileNamesArg.getValue();
    } catch( TCLAP::ArgException &e )
    {
      LOG(ERROR) << "Parsing error: " << e.error() << " for " << e.argId();
    }

    return validate( msg );
  }

  bool validate( stringstream &msg )
  {
    return true;
  }

};




class ExtractMain
{
 public:
  ExtractMain( ExtractOpts &options )
      : opts( options )
  {;}


  int run( void ) {

    for(vector<string>::iterator itr = opts.inFiles.begin();
        itr != opts.inFiles.end(); ++itr ) {

      VideoCapture vid( *itr );

      if( !vid.isOpened() ) {
        cerr << "Couldn't open video source \"" << *itr << "\"" << endl;
        return -1;
      }

      // Compute the output file's name
      boost::filesystem::path p( *itr );
      boost::filesystem::path imgFile( p.stem().replace_extension(".jpg") );

      Mat img;
      vid >> img;

      imwrite( imgFile.string(), img );

    }

  }


  //      if( opts.doDisplay ) namedWindow( "Extract" );
  //
  //      switch( opts.verb ) {
  //        case ExtractOpts::EXTRACT_SINGLE:
  //          return doExtractSingle();
  //          case ExtractOpts::EXTRACT_INTERVAL:
  //return doExtractInterval();
  //        default:
  //          return -1;
  //      }
  //
  //      return -1;
  //    }
  //
  //    int doExtractSingle( void )
  //    {
  //      string videoSource( opts.inFiles[0] );
  //      string imageOut( opts.inFiles[1] );
  //
  //      VideoCapture vid( videoSource );
  //
  //      if( !vid.isOpened() ) {
  //        cerr << "Couldn't open video source \"" << videoSource << "\"" << endl;
  //        return -1;
  //      }
  //
  //      if( opts.seekTo > 0 ) {
  //        int videoLength = vid.get( CV_CAP_PROP_FRAME_COUNT );
  //        if( opts.seekTo > videoLength ) {
  //          cerr << "Requested seek-to of " << opts.seekTo << " after end of video " << videoLength << endl;
  //          return -1;
  //        }
  //
  //        vid.set( CV_CAP_PROP_POS_FRAMES, opts.seekTo );
  //      }
  //
  //      Mat img;
  //      vid.read( img );
  //
  //      cout << "Wrote to \"" << imageOut << "\"" << endl;
  //      imwrite( imageOut.c_str(), img );
  //
  //      return 0;
  //    }
  //
  //    int doExtractInterval( void )
  //    {
  //      string videoSource( opts.inFiles[0] );
  //      fs::path directoryOut( opts.inFiles[1] );
  //
  //      VideoCapture vid( videoSource );
  //
  //      if( !vid.isOpened() ) {
  //        cerr << "Couldn't open video source \"" << videoSource << "\"" << endl;
  //        return -1;
  //      }
  //
  //      double vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );
  //      int digits = ceil( log10( vidLength ) );
  //
  //      if( opts.intervalSeconds > 0 ) opts.intervalFrames = opts.intervalSeconds * vid.get( CV_CAP_PROP_FPS );
  //
  //      if( (opts.intervalFrames > 0) && (vidLength / opts.intervalFrames > 1000) ) {
  //        cerr << "This interval will result in " << vidLength / opts.intervalFrames << "images being extracted.  Please confirm you want this with the '--yes' option." << endl;
  //        return -1;
  //      }
  //
  //
  //      if( !fs::is_directory( directoryOut ) ) fs::create_directories( directoryOut );
  //
  //      Mat img;
  //      while( vid.read( img ) ) {
  //        char filename[40];
  //        int currentFrame = vid.get( CV_CAP_PROP_POS_FRAMES );
  //        snprintf( filename, 39, "frame_%0*d.png", digits, currentFrame );
  //        fs::path fullPath( directoryOut );
  //        fullPath /= filename;
  //
  //        cout << "Output frame " << currentFrame << " to " << fullPath.string() << endl;
  //        imwrite( fullPath.c_str(), img );
  //
  //        if( opts.doDisplay ) {
  //          imshow("Extract",img );
  //          waitKey( opts.waitKey );
  //        }
  //
  //
  //        if( opts.intervalFrames > 1 ) {
  //          int destFrame = currentFrame + opts.intervalFrames - 1;
  //          if( destFrame < vidLength ) {
  //            vid.set( CV_CAP_PROP_POS_FRAMES, currentFrame + opts.intervalFrames - 1 );
  //          } else {
  //            // Done
  //            break;
  //          }
  //        }
  //      }
  //
  //      return 0;
  //
  //
  //    }



private:
ExtractOpts opts;
};



int main( int argc, char **argv ) 
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  ExtractOpts opts;
  stringstream msg;
  if( !opts.parseOpts( argc, argv, msg ) ) {
    cout << msg.str() << endl;
    exit(-1);
  }

  ExtractMain main( opts );

  exit( main.run() );

}

