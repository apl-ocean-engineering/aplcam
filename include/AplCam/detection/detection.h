#pragma once

#include <string>
#include <vector>
#include <array>

#include <opencv2/core/core.hpp>

#include "AplCam/board/board.h"
#include "AplCam/types.h"

#include <kchashdb.h>

namespace AplCam {

using std::string;

struct SharedPoints
{
  ImagePointsVec imagePoints[2];
  ObjectPointsVec worldPoints;
};

struct Detection
{
  Detection( void )
      : points(), corners(), ids()
  { ; }

//  trans(0,0,0), rot(0,0,0), hasTrans(false), hasRot(false) {;}

  virtual ~Detection() {;}

  // bool found;
  ImagePointsVec points;
  ObjectPointsVec corners;
  std::vector< int > ids;

  // float pointSpacing;

  // cv::Vec3f trans, rot;
  // bool hasTrans, hasRot;

  virtual bool good( void ) const { return size() > 0; }
  unsigned int size( void ) const { return points.size(); }

  void add( const ObjectPoint &obj, const ImagePoint &img, const int id )
  {
    corners.push_back( obj );
    points.push_back( img );
    ids.push_back( id );
  }

  // virtual void calculateCorners( const Board &board );
  //virtual void drawCorners(  const Board &board, cv::Mat &view ) const;
  virtual void draw(  cv::Mat &img ) const;

  typedef enum { NO_GOOD_CONFIGURATIONS, NOT_ENOUGH_POINTS = -1, ALL_VALID = 0 } Validate_Return_Code;

  virtual Validate_Return_Code validate( void ) { return ALL_VALID; };

  virtual void writeCache( const Board &board, const std::string &cacheFile ) const;
  virtual std::string serialize( void ) const;
  virtual void serializeToFileStorage( cv::FileStorage &fs ) const;

  cv::Mat boardToImageH( void ) const;

  static Detection *unserialize( const std::string &str );
  static Detection *loadCache( const std::string &cacheFile );
  static Detection *unserializeFromFileStorage( const cv::FileStorage &fs );
  static SharedPoints sharedWith( const Detection &a,  const Detection &b );

};
}
