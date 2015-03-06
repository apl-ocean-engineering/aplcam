
#include <stdlib.h>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <getopt.h>

#include <gsl/gsl_randist.h>
#include <gsl/gsl_cdf.h>

#ifdef USE_FFTS
#include <ffts.h>
#endif


#include "file_utils.h"
#include "trendnet_time_code.h"

#include "video.h"

using namespace cv;
using namespace std;

#define MAKE_NORMFILE
#ifdef MAKE_NORMFILE
ofstream normFile;
#endif


struct AlignmentOptions
{
  // n.b. the default window should be a non-round number so you don't get an ambiguous number of second transitions...
  AlignmentOptions( void )
    : window( 4.2 ), maxDelta( 5.0 ), lookahead(1.2), offset(0), offsetGiven(false)
  {;}


  float window, maxDelta, lookahead;
  int offset;
  bool offsetGiven;

  string video1, video2;

  bool parseArgv( int argc, char **argv, string &msg )
  {
    static struct option long_options[] = {
      { "window", true, NULL, 'w' },
      { "max-delay", true, NULL, 'd'},
      { "offset", true, NULL, 'o'},
      { "lookahead", true, NULL, 'l'},
      { "help", false, NULL, '?' },
      { 0, 0, 0, }
    };

    int indexPtr;
    int optVal;
    while( (optVal = getopt_long( argc, argv, ":w:d:o:l:?", long_options, &indexPtr )) != -1 ) {
      switch(optVal) {
        case 'd':
          maxDelta = atof(optarg);
          break;
        case 'w':
          window = atof( optarg );
          break;
        case 'o':
          offset = atoi( optarg );
          offsetGiven = true;
          break;
        case 'l':
          lookahead = atof( optarg );
          break;
        case '?':
          help( msg );
          return false;
          break;
        default:
          stringstream strm;
          strm << "Unknown option \'" << optopt << "\'";
          msg = strm.str();
          return false;
          break;
      }

    }

    if( (argc - optind) < 2 ) {
      msg = "Must specify two video files on command line";
      return false;
    }

    video1 = argv[optind++];
    video2 = argv[optind];

    return validate( msg );
  }

  bool validate( string &msg )
  {
    if( !file_exists( video1 ) ) {
      msg = "File \'" + video1 + "\' doesn't exist";
      return false;
    }
    if( !file_exists( video2 ) ) {
      msg = "File \'" + video2 + "\' doesn't exist";
      return false;
    }

    return true;
  }

  void help( string &msg )
  {
    stringstream strm;
    strm << "Help!" << endl;

    msg = strm.str();
  }
};



class Synchronizer
{
  public:
    Synchronizer( Video &v0, Video &v1 )
      : _video0( v0 ), _video1( v1 ), _offset( 0 )
    {;}

    void setOffset( int offset )
    { _offset = offset; }

    void rewind( void )
    {
      if( _offset < 0 ) {
        _video0.seek( -_offset );
        _video1.seek( 0 );
      } else {
        _video0.seek( 0 );
        _video1.seek( _offset );
      }
      cout << "Rewind to frames: " << _video0.frame() << ' ' << _video1.frame() << endl;
    }

    bool seek( int which, int dest )
    {
      if( which == 0 ) {
        int dest1 = dest + _offset;

        if( dest >= 0 && dest < _video0.frameCount() && 
            dest1 >= 0 && dest1 < _video1.frameCount() ) {
          _video0.seek(dest);
          _video1.seek(dest1);
          return true;
        } 

      } else {
        return seek( 0, dest - _offset );
      }

      return  false;
    }

    bool scrub( int offset )
    {  int dest0 = _video0.frame() + offset;
      return seek( 0, dest0 );
    }

    static const float Scale;

    bool advanceToNextTransition( int which )
    {
      int current = (which == 0) ? _video0.frame() : _video1.frame();

      const Video::TransitionVec &transitions( (which==0) ? _video0.transitions() : _video1.transitions() );

      if( transitions.size() == 0 ) 
        return false;
      else 
      {
        if( transitions[0].frame > current ) {
          seek( which, transitions[0].frame );
          cout << "Advancing video " << which << " to frame " << transitions[0].frame << endl;
          return true;
        } 

        if( transitions.size() > 1 )
          for( int i = 1; i < transitions.size(); ++i ) {
            if( (transitions[i-1].frame <= current) && (transitions[i].frame > current) ) {
              cout << "Advancing video " << which << " to frame " << transitions[i].frame << endl;
              seek( which , transitions[i].frame );
              return true;
            }
          }
      }

      return false;
    }

    Size compositeSize( void )
    { return  Size( Scale*(_video0.width() + _video1.width()), Scale*std::max(_video0.height(), _video1.height()) ); }

    void advanceOnly( int which )
    {
      if( which == 0 ) {
        _offset--;
        _video1.scrub(-1);
      } else {
        _offset++;
        _video0.scrub(-1);
      }
    }

    bool nextCompositeFrame( Mat &img )
    {

      img.create( compositeSize(), CV_8UC3 );

      Mat video0ROI( img, Rect( 0, 0, Scale*_video0.width(), Scale*_video0.height() ) );
      Mat video1ROI( img, Rect( Scale*_video0.width(), 0, Scale*_video1.width(), Scale*_video1.height() ) );

      cout << "Frames: " << _video0.frame() << ' ' << _video1.frame() <<  ' ' << _offset << endl;

      Mat frame;
      if( _video0.read( frame ) ) 
        if( Scale != 1.0 )
          resize( frame, video0ROI, video0ROI.size() );
        else
          frame.copyTo( video0ROI );
      else return false;

      if( _video1.read(frame )  )
        if( Scale != 1.0 )
          resize( frame, video1ROI, video1ROI.size() );
        else
          frame.copyTo( video1ROI );
      else return false;

      return true;
    }

    // Tools for estimating initial offset
    float compareSpans( IndexPair &thisSpan, IndexPair &otherSpan )
    {
      cout << "======================" << endl;
      cout << "this span:  " << thisSpan.first << ' ' << thisSpan.second << endl;
      for( int i = thisSpan.first; i < thisSpan.second; ++i ) {
        if( i > thisSpan.first ) 
          cout << _video0.transitions()[i].frame << ' ' << _video0.transitions()[i].frame - _video0.transitions()[i-1].frame << endl;
        else
          cout << _video0.transitions()[i].frame << ' ' <<endl;
      }

      cout << "other span: " << otherSpan.first << ' ' << otherSpan.second << endl;
      for( int i = otherSpan.first; i < otherSpan.second; ++i ) {
        if( i > otherSpan.first ) 
          cout << _video1.transitions()[i].frame << ' ' << _video1.transitions()[i].frame - _video1.transitions()[i-1].frame << endl;
        else
          cout << _video1.transitions()[i].frame << ' ' <<endl;
      }


      if( (thisSpan.second - thisSpan.first) != (otherSpan.second - otherSpan.first) ) {
        cout << "Spans are different lengths, using shorter of the two " << endl;
      }
      int  length = std::min( thisSpan.second-thisSpan.first, otherSpan.second-otherSpan.first );

      float total = 0.0;
      for( int i = 0; i < length; ++i ) {
        total += norm( _video0.transitions()[ thisSpan.first+i ].before, _video1.transitions()[ otherSpan.first+i ].before, NORM_L2 );
        total += norm( _video0.transitions()[ thisSpan.first+i ].after, _video1.transitions()[ otherSpan.first+i ].after, NORM_L2 );
      }
      total /= length;

      return total;
    }

    struct OffsetResult {
      OffsetResult( IndexPair vid0, IndexPair vid1 )
        : v0( vid0 ), v1( vid1 )
      {;}

      IndexPair v0, v1;
    };

    int estimateOffset( float window, float maxDelta ) 
    {
      // TODO:  Currently assumes both videos have same FPS
      int maxDeltaFrames = floor(maxDelta *  _video0.fps()),
          windowFrames = floor(window * _video0.fps());
      map <float, OffsetResult> results;

      IndexPair thisSpan( _video0.getSpan( 0, windowFrames ) );
      IndexPair otherSpan( _video1.getSpan( maxDeltaFrames, windowFrames ) );

      cout << "Getting span with max delta " << maxDeltaFrames << "  length " << windowFrames << endl;
      cout << " Got " << thisSpan.first << ' ' << thisSpan.second << endl;
      cout << " Got " << otherSpan.first << ' ' << otherSpan.second << endl;


      // Start with set of transitions from maxDelta to maxDelta+window (this is the maximum backwards shift on video1)

      do {
        float result = compareSpans( thisSpan, otherSpan );
        cout << "    result: " << result << endl;
        results.insert( make_pair( result, OffsetResult( thisSpan, otherSpan ) ) );
      } while( _video1.shiftSpan( otherSpan, windowFrames, -1 ) ) ;

      // Now start shifting my span forward
      do {
        float result = compareSpans( thisSpan, otherSpan );
        cout << "    result: " << result << endl;
        results.insert( make_pair( result, OffsetResult( thisSpan, otherSpan ) ) );
      } while( _video0.shiftSpan( thisSpan, windowFrames, +1 ) );

      OffsetResult best = results.begin()->second;

      // Calculate offset from end of spans...
      _offset = _video1.transitions()[ best.v1.second-1 ].frame - _video0.transitions()[ best.v0.second-1 ].frame;

      cout << "Best alignment has score " << results.begin()->first << endl;
      cout << "With frames " << _video0.transitions()[ best.v0.second-1 ].frame << " " << _video1.transitions()[ best.v1.second-1 ].frame << endl;
      cout << "With video1 offset to video0 by " << _offset << endl;

      return _offset;
    }



  private:
    Video & _video0, &_video1;

    int _offset;


};

const float Synchronizer::Scale = 1;


int main( int argc, char **argv )
{
  string error;
  AlignmentOptions opts;
  if( opts.parseArgv( argc, argv, error ) != true ) {
    if( !error.empty() ) cout << error  << endl;
    exit(-1);
  }

#ifdef MAKE_NORMFILE
  normFile.open("norms.txt");
#endif

  VideoLookahead video[2] = { VideoLookahead( opts.video1, opts.lookahead ), VideoLookahead( opts.video2, opts.lookahead ) };

  for( int i = 0; i < 2; ++i ) {
    if( !video[i].capture.isOpened() ) {
      cerr << "Can't open video " << i << endl;
      exit(-1);
    }

    cout << video[i].dump() << endl;

    video[i].findTransitionsSeconds( 0, 2*opts.maxDelta );

    stringstream filename;
    filename << "/tmp/transitions_" << i << ".png";

    video[i].dumpTransitions( filename.str() );

    if( i == 0 ) normFile << endl << endl;
  }

#ifdef MAKE_NORMFILE
  normFile.close();
#endif

  Synchronizer sync( video[0], video[1] );
  if( opts.offsetGiven )
    sync.setOffset( opts.offset );
  else
    sync.estimateOffset( opts.window, opts.maxDelta );

  sync.rewind();
  sync.seek( 0, 151 );

  Mat img;
  while( sync.nextCompositeFrame( img )) {
    int ch;
    imshow( "Composite", img );
    ch = waitKey( 0 );

    if( ch == 'q' )
      break;
    else if (ch == ',')
      sync.scrub(-2);
    else if (ch == '[')
      sync.advanceToNextTransition( 0 );
    else if (ch == ']')
      sync.advanceToNextTransition( 1 );
    else if (ch == 'R')
      sync.rewind();
    else if (ch == 'l')
      sync.advanceOnly( 0 );
    else if (ch == 'r')
      sync.advanceOnly( 1 );

  }


  exit(0);
}





#ifdef FOURIER_APPROACH

int normsSeconds( float start, float end, float **n )
{  return norms( floor( start * fps() ), ceil( end * fps() ), n ); }

int norms( int start, int end, float **n, int length = 0 )
{
  start = std::max( 0, std::min( frameCount(), start ) );
  end = std::max( 0, std::min( frameCount(), end ) );

  int window = end-start;
  int len = std::max( length, window );

  *n = (float *)valloc( len * sizeof(float) );

  seek( start );

  Mat prev;
  for( int at = start; at < end; ++at ) {
    Mat fullImage;
    capture >> fullImage;

    Mat timeCodeROI( fullImage, TimeCodeROI );

    if( prev.empty() ) {
      cv::cvtColor( timeCodeROI, prev, CV_BGR2GRAY );
      continue;
    }

    Mat curr;
    cv::cvtColor( timeCodeROI, curr, CV_BGR2GRAY );

    (*n)[2*at] = cv::norm( prev, curr, NORM_L2 ); 
    (*n)[2*at+1] = 0.0f;

    prev = curr;
  }

  // Zero pad the remainder
  memset( &(*n[window]), 0, (len - window) * sizeof(float) );

  return window;
}

// Should be window floor'ed to nearest power of two or somesuch
int windowLength = 128;
float *norms0, *norms1;
int count0 = video[0].norms( 0, windowLength, &norms0, windowLength*2 );
int count1 = video[1].norms( 0, windowLength, &norms1, windowLength*2 );

cout << "Got " << count0 << ", " << count1 << " norms" << endl;

ffts_plan_t *forward0 = ffts_init_1d_real( 2*windowLength, -1 );
ffts_plan_t *forward1 = ffts_init_1d_real( 2*windowLength, -1 );
ffts_plan_t *backward = ffts_init_1d_real( 2*windowLength, 1 );

// Real-to-complex transforms return (N/2 + 1) complex numbers
// But I've zero-padded the input to 2*windowLength
float fourierLength = (windowLength + 1);
float __attribute__ ((aligned(32))) *fourier0 = (float *)valloc( fourierLength * 2 * sizeof(float) );
float __attribute__ ((aligned(32))) *fourier1 = (float *)valloc( fourierLength * 2 * sizeof(float) );
float __attribute__ ((aligned(32))) *fourierRes = (float *)valloc( fourierLength * 2 * sizeof(float) );
float __attribute__ ((aligned(32))) *result = (float *)valloc( windowLength*2 * sizeof(float) );

ffts_execute( forward0, norms0, fourier0 );
ffts_execute( forward0, norms1, fourier1 );

// Convolve
float scale = 1.0 / fourierLength;
for( int i = 0; i < fourierLength; ++i ) {
  fourierRes[ 2*i ]   = (fourier0[ 2*i ]*fourier1[2*i]   + fourier0[2*i+1]*fourier1[2*i+1]) * scale;
  fourierRes[ 2*i+1 ] = (fourier0[ 2*i+1 ]*fourier1[2*i] - fourier0[2*i]*fourier1[2*i+1]) * scale;
}

ffts_execute( backward, fourierRes, result );

ofstream fst("correlation.txt");

float max = 0;
int maxIdx = -1;
for( int i = 0; i < 2*windowLength; ++i ) {
  if( maxIdx < 0 || result[i] >  max ) {
    maxIdx = i;
    max = result[i];
  }

  fst << i << " " << result[i] << endl;
}

fst.close();

cout << "Max occurs are index " << maxIdx << " value " << max << endl;

ffts_free( forward0 );
ffts_free( forward1 );
ffts_free( backward );

free( fourier0 );
free( fourier1 );
free( fourierRes );
free( result );


free( norms0 );
free( norms1 );

// Try a cross-correlation based approach (how will this deal with missing data?)


#endif