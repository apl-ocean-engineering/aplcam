#ifndef __CALIBRATION_DB_H__
#define __CALIBRATION_DB_H__

#include <string>
#include <kchashdb.h>

#include "calibration_serializer.h"

namespace AplCam {

  using std::string;
  using kyotocabinet::BasicDB;
  using kyotocabinet::HashDB;

  class CalibrationDb {
    public:

      CalibrationDb( void );

      CalibrationDb( const string &filename, bool writable = false );

      bool open( const string &filename, bool writable = false );


      kyotocabinet::BasicDB::Error error( void ) { return _db.error(); }
      kyotocabinet::DB::Cursor *cursor( void ) { return _db.cursor(); }

      bool isOpened( void ) { return (_db.flags() & HashDB::FOPEN); }
      uint8_t flags( void ) { return _db.flags(); }

      bool save( const string &key, const CalibrationSerializer &ser );

      bool get( const string &key, string * value )
      { return _db.get( key, value ); }

      bool has( const string &key );

      void findKeysStartingWith( const string &val, vector< string > &keys );
      

    protected:
      uint32_t modeFlags( bool writable );

      HashDB _db;
  };



}

#endif
