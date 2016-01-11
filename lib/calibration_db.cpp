
#include "AplCam/calibration_db.h"

namespace AplCam {


  using kyotocabinet::DB;

  //===========================================================================
  // CalibrationDb
  //===========================================================================

  CalibrationDb::CalibrationDb( void )
    : _db()
  {;}

  CalibrationDb::CalibrationDb( const string &filename, bool writable )
    : _db()
  {
    _db.open( filename, modeFlags( writable ) );
  }

  bool CalibrationDb::open( const string &filename, bool writable )
  {
    return  _db.open( filename, modeFlags( writable ) );
  }

  uint32_t CalibrationDb::modeFlags( bool writable )
  {
    return writable ?
      ( HashDB::OWRITER | HashDB::OCREATE ) :
      ( HashDB::OREADER | HashDB::ONOLOCK );
  }


  bool CalibrationDb::has( const string &key )
  {
    return (_db.check( key ) != -1 );
  }

  bool CalibrationDb::save( const string &key, const CalibrationSerializer &ser )
  {
    string str;
    ser.serialize( str );
    if( !_db.set( key,  str ) ) return false;
    return true;
  }


  void CalibrationDb::findKeysStartingWith( const string &val, vector <string> &keys )
  {
    kyotocabinet::DB::Cursor *cur = _db.cursor();
    cur->jump();

    keys.clear();
    string thisKey;
    int len = val.size();
    while( cur->get_key( &thisKey, true ) ) {
      if( thisKey.compare( 0, len, val ) == 0 ) keys.push_back( thisKey );
    }
  }


}
