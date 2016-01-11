#ifndef __DETECTION_DB_H__
#define __DETECTION_DB_H__

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "AplCam/detection/detection.h"

#include <kchashdb.h>


using kyotocabinet::HashDB;

namespace AplCam {
  class DetectionSet;

  class DetectionDb {
    public:

      DetectionDb();
      DetectionDb( const string &dbFile, bool writer = false );

      ~DetectionDb();

      bool open( const string &dbFile, bool writer = false );
      bool open( const string &dbDir, const string &videoFile, bool writer = false );

      bool save( const int frame, const Detection &detection );
      bool save( const DetectionSet &detSet );

      bool has( const int frame );
      bool has( const string &key );
      bool has_meta( void );

      bool update( const int frame, const Detection &detection );
      bool update( const string &key, const Detection &detection );

      Detection *load( const int frame );
      Detection *load( const string &key );

      kyotocabinet::BasicDB::Error error( void ) { return _db.error(); }

      int maxKey( void );

      static const std::string FrameToKey( const int frame );

      kyotocabinet::DB::Cursor *cursor( void );

      cv::Size imageSize( void ) const { return _imageSize; }
      int vidLength( void ) const { return _vidLength; }
      float fps( void ) const { return _fps; }

      bool setMeta( unsigned int length, int width, int height, float fps );

      static const string MetaKey,
                          MetaFpsKey,
                          MetaWidthKey,
                          MetaHeightKey,
                          MetaLengthKey;

    protected:

      bool saveMeta( void );
      void loadMeta( void );

      HashDB _db;
      kyotocabinet::DB::Cursor *_cursor;

      // Metadata
      cv::Size _imageSize;
      int _vidLength;
      float _fps;

  };
}

#endif
