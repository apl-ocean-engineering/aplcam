
#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "board.h"
#include "detection.h"

using namespace std;
using namespace cv;
using namespace AplCam;

Detection *Board::detectPattern( const Mat &gray, vector< Point2f > &pointbuf )
{
  Detection *detect = new Detection();

  switch( pattern )
  {
    case CHESSBOARD:
      detect->found = findChessboardCorners( gray, size(), detect->points,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

      // improve the found corners' coordinate accuracy
      if( detect->found) cornerSubPix( gray, detect->points, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

      break;
    case CIRCLES_GRID:
      detect->found = findCirclesGrid( gray, size(), detect->points );
      break;
    case ASYMMETRIC_CIRCLES_GRID:
      detect->found = findCirclesGrid( gray, size(), detect->points, CALIB_CB_ASYMMETRIC_GRID );
      break;
    default:
      cerr << "Unknown pattern type" << endl;
      return NULL;
  }

  detect->calculateCorners( *this );
  return detect;
}

    Detection *Board::detectPattern( const cv::Mat &gray )
{
  vector< Point2f > pointbuf;
  return detectPattern( gray, pointbuf );
}


Board *Board::load( const string &infile, const string &name )
{
  FileStorage fs( infile, FileStorage::READ );
  if( ! fs.isOpened() ) {
    cout << "Couldn't open board file \"" << infile << "\"" << endl;
    exit(-1);
  }

  string type_s;
  int width, height;
  float squares;
  //Pattern type;

  fs["type"] >> type_s;
  fs["width"] >> width;
  fs["height"] >> height;
  fs["squareSize"] >> squares;

  Board *board = NULL;
  if( type_s.compare("chessboard" ) == 0 ) {
    board = new Board( CHESSBOARD, width, height, squares, name );
  } else if( type_s.compare("hough_circle" ) == 0 ) {
    board = new HoughCircleBoard( name );
  } else if( type_s.compare("apriltags_36h11" ) == 0) {
#ifdef USE_APRILTAGS
    board = new AprilTagsBoard( width, height, squares, name );
#else
    cout << "Not compiled for Apriltags." << endl;
#endif
  } else {
    cout << "Don't know how to handle board type \"" << type_s << "\"" << endl;
  }

  board->loadCallback( fs );

  return board;
}

cv::Point3f Board::worldLocation( const cv::Point2i &xy ) const
{
  Point3f halfSize( squareSize * size().width / 2.0, squareSize * size().height / 2.0, 0 );
  return Point3f( xy.x * squareSize, xy.y * squareSize, 0 ) - halfSize;
}

ObjectPointsVec Board::corners( void ) // const CornersReference ref )
{
  Point3f halfSize( squareSize * size().width / 2.0, squareSize * size().height / 2.0, 0 );

  ObjectPointsVec out;
  for( int x = 0; x < width; ++x ) 
    for( int y = 0; y < height; ++y ) 
      //if( ref == BOARD_UL ) 
      //  out.push_back( worldLocation( Point2i( x, y ) ) );
      //else
        out.push_back( worldLocation( Point2i(x,y) ) );

  return out;
}

std::vector< int > Board::ids( void )
{
  return vector< int >();
}

void Board::extents( ObjectPointsVec &vec ) const
{
  vec.resize(4);
  vec[0] = worldLocation( Point2i( 0,0 ) );
  vec[1] = worldLocation( Point2i( width-1,0 ) );
  vec[2] = worldLocation( Point2i( width-1,height-1 ) );
  vec[3] = worldLocation( Point2i( 0,height-1 ) );
}



//===========================================================================
//  HoughCircleBoard
//===========================================================================

void HoughCircleBoard::loadCallback( FileStorage &fs )
{
  //fs["ids"] >> _ids;
}

Detection *HoughCircleBoard::detectPattern( const cv::Mat &img )
{
  // Pull out color
Scalar targetColor( 0, 128, 255 );

namedWindow("channel0");
namedWindow("channel1");
namedWindow("channel2");

namedWindow("h0");
namedWindow("h1");

float alpha = 0.5 * ( 2* targetColor[2]/256 - targetColor[1]/256 - targetColor[0]/256 );
float beta = sqrt(3)/2 * ( targetColor[1]/256 - targetColor[0]/256 );
float ang = atan2( beta, alpha );

Scalar targetCS( cos( ang ), sin( ang ) );

Mat flt, hsv;
img.convertTo( flt, CV_32FC3 );
cvtColor( flt, hsv, CV_BGR2HSV );
vector<Mat> channels;
split( hsv, channels );

imshow("channel0", channels[0] / 360 );
imshow("channel1", channels[1] );
imshow("channel2", channels[2] );


// Make cos and sin matrices 
Mat h[2];
polarToCart( Mat(), channels[0] * M_PI/180.0, h[0], h[1] );
Mat hue;
merge( h, 2, hue );

imshow("h0", h[0] );
imshow("h1", h[1] );

// Make cos and sin from target
Mat targetHue( hue.size(), hue.type(), targetCS );
Mat diff = hue - targetHue;

Mat gray;
cvtColor( img, gray, CV_BGR2GRAY );
//dot.convertTo( gray, CV_8UC1 );
//cout << gray;
//imshow( "gray", gray );

waitKey(0);

  Mat blurred;
  GaussianBlur( gray, blurred, Size(9,9), 2,2 );

  vector<Vec3f> circles;
  HoughCircles( blurred, circles, HOUGH_GRADIENT, 2, gray.rows/4, 200, 100 );

  return new HoughCircleDetection( circles );
}


//===========================================================================
//  HoughCircleBoard
//===========================================================================

void CircleGridBoard::loadCallback( FileStorage &fs )
{
}

Detection *CircleGridBoard::detectPattern( const cv::Mat &gray, vector< cv::Point2f > &pointbuf )
{
  Detection *detect = new Detection();
  detect->found = findCirclesGrid( gray, size(), detect->points );

  detect->calculateCorners( *this );
  return detect;
}


#ifdef USE_APRILTAGS
//===========================================================================
//  AprilTagsBoard
//===========================================================================

//const int AprilTagsBoard::_ids[] = { 0,1,2,3,4,5,6,
//  24,25,26,27,28,29,30,
//  48,49,50,51,52,53,54,
//  72,73,74,75,76,77,78,
//  96,97,98,99,100,101,102 };

void AprilTagsBoard::loadCallback( FileStorage &fs )
{
  fs["ids"] >> _ids;
}

Detection *AprilTagsBoard::detectPattern( const cv::Mat &gray, vector< cv::Point2f > &pointbuf )
{
  AprilTags::TagDetector tagDetector( _tagCode );

  vector<AprilTags::TagDetection> detections = tagDetector.extractTags(gray);
  //cout << "found " << detections.size() << " tags:" << endl;

  AprilTagsDetection *detect = new AprilTagsDetection( detections );
  detect->calculateCorners( *this );
  return detect;
}

bool AprilTagsBoard::find( const int id, cv::Point2i &xy  ) const
{
  for( int x = 0; x < width; ++x ) 
    for( int y = 0; y < height; ++y ) 
      if( _ids.at<int>(y,x) == id ) 
      {
        xy.x = x; xy.y = y;
        return true;
      }

  return false;
}

std::vector< int > AprilTagsBoard::ids( void )
{
  vector< int > out;
  for( int x = 0; x < width; ++x ) 
    for( int y = 0; y < height; ++y ) 
      out.push_back( _ids.at< int >( y, x ) );

  return out;
}

#endif
