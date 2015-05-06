
#include <opencv2/calib3d/calib3d.hpp>
#include "detection.h"
#include "file_utils.h"

using namespace std;
using namespace cv;

using namespace AplCam;

void Detection::calculateCorners( const Board &board )
{ 
  corners.resize(0);

  switch(board.pattern)
  {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for( int i = 0; i < board.size().height; i++ )
        for( int j = 0; j < board.size().width; j++ )
          corners.push_back(Point3f(float(j*board.squareSize),
                float(i*board.squareSize), 0));
      break;

    case ASYMMETRIC_CIRCLES_GRID:
      for( int i = 0; i < board.size().height; i++ )
        for( int j = 0; j < board.size().width; j++ )
          corners.push_back(Point3f(float((2*j + i % 2)*board.squareSize),
                float(i*board.squareSize), 0));
      break;

    default:
      CV_Error(CV_StsBadArg, "Unknown pattern type\n");
  }
}

void Detection::drawCorners( const Board &board, Mat &view ) const
{
  cout << "Drawing " << corners.size() << " corners" << endl;
  drawChessboardCorners( view, board.size(), Mat(points), found );
}

Mat Detection::boardToImageH( void ) const
{
  // Calculate homography from board to image, then project board's corners into image

  // Need to flatten corners to points
  ImagePointsVec crn;
  for( size_t i = 0; i < corners.size(); ++i ) crn.push_back( ImagePoint( corners[i][0], corners[i][1] ) );
  return findHomography( crn, points );
}




//============================================================================
//  Serialization/unserialization methods
//============================================================================

void Detection::serialize( string &str ) const
{
  FileStorage fs("foo.yml", FileStorage::WRITE | FileStorage::MEMORY );
  serializeToFileStorage( fs);
  str = fs.releaseAndGetString();
}

void Detection::writeCache( const Board &board, const string &cacheFile ) const
{
  mkdir_p( cacheFile );

  FileStorage fs( cacheFile, FileStorage::WRITE );
  fs << "board_type" << board.patternString();
  fs << "board_name" << board.name;
  serializeToFileStorage( fs );
}

void Detection::serializeToFileStorage(  FileStorage &fs ) const
{
  fs << "image_points" << Mat( points );
  fs << "world_points" << Mat( corners );
  fs << "ids" << Mat( ids );

  fs << "t" << trans;
  fs << "R" << rot;
}


Detection *Detection::unserialize( const string &str )
{
  FileStorage fs( str, FileStorage::READ | FileStorage::MEMORY );
  return unserializeFromFileStorage( fs );
}

Detection *Detection::loadCache( const string &cacheFile )
{
  if( !file_exists( cacheFile ) ) {
    cout << "Unable to find cache file \"" << cacheFile << "\"" << endl;
    return NULL;
  }

  FileStorage fs( cacheFile, FileStorage::READ );
  return unserializeFromFileStorage( fs );
}

Detection *Detection::unserializeFromFileStorage( const FileStorage &fs )
{

  Detection *detection = new Detection();

  // Load and validate data
  Mat pts;
  fs["image_points"] >> pts;

  // Should be able to do this in-place, but ...
  for( int i = 0; i < pts.rows; ++i ) {
    detection->points.push_back( Point2f(pts.at<float>(i,0), pts.at<float>(i,1) ) );
  }

  fs["world_points"] >> pts;

  // Should be able to do this in-place, but ...
  for( int i = 0; i < pts.rows; ++i ) {
    detection->corners.push_back( Point3f(pts.at<float>(i,0), pts.at<float>(i,1), pts.at<float>(i,2) ) );
  }

  fs["ids"] >> pts;
  for( int i = 0; i < pts.rows; ++i ) {
    detection->ids.push_back( pts.at<int>(i,0) );
  }

  if( !fs["t"].empty() ) {
    fs["t"] >> detection->trans; 
    detection->hasTrans = true; 
  }

  if( !fs["R"].empty() ) {
    fs["R"] >> detection->rot;
    detection->hasRot = true;
  }

  return detection;
}

SharedPoints Detection::sharedWith( Detection &a, Detection &b )
{
  SharedPoints shared;

  for( int i = 0; i < a.size(); ++i ) {
    for( int j = 0; j < b.size(); ++j ) {

      if( a.ids[i] == b.ids[j] ) {
        //      cout << "Comparing " << a.ids[i] << " to " << b.ids[j] << endl;
        //      cout << a.points[i] << b.points[j] << a.corners[i] << endl;
        shared.imagePoints[0].push_back( a.points[i] );
        shared.imagePoints[1].push_back( b.points[j] );

        assert( a.corners[i] == b.corners[j] );
        shared.worldPoints.push_back( a.corners[i] );
        break;
      }
    }
  }

  return shared;
}

//============================================================================
// HoughCircleDetection
//============================================================================

void HoughCircleDetection::drawCorners( const Board &board, Mat &view ) const
{
  for( size_t i = 0; i < _circles.size() ; ++i ) {
    circle( view, Point( _circles[i][0], _circles[i][1] ), _circles[i][2], Scalar( 0, 0, 255 ), 2 );
  }
}



#ifdef USE_APRILTAGS

//============================================================================
//  AprilTagDetection
//============================================================================

void AprilTagsDetection::calculateCorners( const AprilTagsBoard &board )
{
  // Go for a simple model here, assume all tags are unique on the board

  points.clear();
  corners.clear();
  ids.clear();

  for( size_t i = 0; i < _det.size(); ++i ) {

    Point2i loc;
    if( board.find( _det[i].id, loc ) ) {
      //cout << "Found  tag id " << _det[i].id << endl;

      points.push_back( Point2f( _det[i].cxy.first, _det[i].cxy.second ) );
      corners.push_back( board.worldLocation( loc ) );
      ids.push_back( _det[i].id );

    } else {
      cerr << "Couldn't find tag \'" << _det[i].id << "\'" << endl;
    }
  }
}

#endif

