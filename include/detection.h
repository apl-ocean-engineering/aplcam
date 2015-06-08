#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "board.h"
#include "distortion_model.h"

#include <kchashdb.h>

namespace AplCam {

using std::string;

struct SharedPoints
{
  Distortion::ImagePointsVec imagePoints[2];
  Distortion::ObjectPointsVec worldPoints;
};

struct Detection 
{
  Detection(  )
      : found(false), points(), corners(), ids(), trans(0,0,0), rot(0,0,0), hasTrans(false), hasRot(false) {;}

  virtual ~Detection() {;}

  bool found;
  Distortion::ImagePointsVec points;
  Distortion::ObjectPointsVec corners;
  std::vector< int > ids;

  cv::Vec3f trans, rot;
  bool hasTrans, hasRot;

  int size( void ) const { return points.size(); }

  void add( const Distortion::ObjectPoint &obj, const Distortion::ImagePoint &img, const int id )
  {
    corners.push_back( obj );
    points.push_back( img );
    ids.push_back( id );
  }

  virtual void calculateCorners( const Board &board );
  virtual void drawCorners(  const Board &board, cv::Mat &view ) const;

  typedef enum { NOT_ENOUGH_POINTS = -1, ALL_VALID = 0 } Validate_Return_Code;

  virtual Validate_Return_Code validate( void ) { return ALL_VALID; };

  virtual void writeCache( const Board &board, const std::string &cacheFile ) const;
  virtual void serialize( std::string &str ) const; 
  virtual void serializeToFileStorage( cv::FileStorage &fs ) const;

  cv::Mat boardToImageH( void ) const;

  static Detection *unserialize( const std::string &str );
  static Detection *loadCache( const std::string &cacheFile );
  static Detection *unserializeFromFileStorage( const cv::FileStorage &fs );
  static SharedPoints sharedWith( const Detection &a,  const Detection &b );

};
}

#endif
