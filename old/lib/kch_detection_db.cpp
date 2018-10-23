
#include <iostream>

#include "AplCam/kch_detection_db.h"
#include "AplCam/detection_set.h"

#include "AplCam/file_utils.h"

using namespace std;
using namespace cv;

namespace AplCam {

  const string KchDetectionDb::MetaKey = "meta",
        KchDetectionDb::MetaFpsKey = "fps",
        KchDetectionDb::MetaWidthKey = "width",
        KchDetectionDb::MetaHeightKey = "height",
        KchDetectionDb::MetaLengthKey = "length";



  //============================================================================
  //  KchDetectionDb
  //============================================================================

  KchDetectionDb::KchDetectionDb( void )
    : _db(), _cursor(NULL), _imageSize( 0, 0), _vidLength(0), _fps(1.0)
  {;}

  KchDetectionDb::KchDetectionDb( const string &dbFile, bool writer )
    : _db(), _cursor(NULL), _imageSize( 0,0 ), _vidLength(0), _fps(1.0)
  {
    open( dbFile, writer );
  }



  KchDetectionDb::~KchDetectionDb()
  {
    _db.close();
    // if( _cursor ) delete _cursor;
  }

  bool KchDetectionDb::open( const string &dbDir, const string &videoFile, bool writer )
  {
    string dbfile = dbDir + "/" + fileHashSHA1( videoFile ) + ".kch";
    return open( dbfile, writer );
  }

  bool KchDetectionDb::open( const string &dbFile, bool writer )
  {
    int flags = (writer==true) ? (HashDB::OWRITER | HashDB::OCREATE) : (HashDB::OREADER);
    if( !_db.open( dbFile, flags ) ) return false;

    loadMeta();

    return true;
  }

  bool KchDetectionDb::save( const int frame, const Detection &detection )
  {
    string str( detection.serialize( ) );
    if( !_db.set( FrameToKey( frame ),  str ) ) return false;
    return true;
  }

  bool KchDetectionDb::save( const DetectionSet &detSet )
  {
    size_t end = detSet.size();
    for( size_t i = 0; i < end; ++i ) save( detSet[i].frame, detSet[i] );

    return true;
  }

  bool KchDetectionDb::has( const int frame )
  {
    return has( FrameToKey( frame ) );
  }

  bool KchDetectionDb::has( const string &key )
  {
    return (_db.check( key ) != -1 );
  }

  bool KchDetectionDb::has_meta( void )
  {
    return has( MetaKey );
  }

  bool KchDetectionDb::update( const int frame, const Detection &detection )
  {
    return update( FrameToKey( frame ), detection );
  }


  bool KchDetectionDb::update( const string &key, const Detection &detection )
  {
    string str(detection.serialize());
    if( !_db.set( key,  str ) ) return false;
    return true;
  }

  Detection *KchDetectionDb::load( const int frame )
  {
    return load( FrameToKey( frame ) );
  }

  Detection *KchDetectionDb::load( const string &key )
  {
    if( ! has(key) ) return NULL;

    string value;
    _db.get( key, &value );
    return Detection::unserialize( value );
  }

  kyotocabinet::DB::Cursor *KchDetectionDb::cursor( void )
  {
    if( _cursor == NULL ) {
      _cursor = _db.cursor();
      _cursor->jump();
    }

    return _cursor;
  }

  struct MaxFinder : public kyotocabinet::DB::Visitor
  {
    MaxFinder( void )
      : _max(0) {;}

    int max( void ) const { return _max; };

    virtual const char *visit_full( const char *kbuf, size_t ksiz, const char *vbuf, size_t vsiz, size_t *sp )
    {
      if( 0 != strncmp( kbuf, KchDetectionDb::MetaKey.c_str(), std::min(ksiz,(size_t)4) ) )
	_max = std::max( _max, (int)atoi( kbuf ) );
      return Visitor::NOP;
    }

    int _max;
  };

  int KchDetectionDb::maxKey( void )
  {
    MaxFinder maxFinder;

    // false == readonly
    _db.iterate( &maxFinder, false );

    return maxFinder.max();
  }

  const string KchDetectionDb::FrameToKey( const int frame )
  {
    const int strWidth = 20;
    char frameKey[strWidth];
    snprintf( frameKey, strWidth-1, "%d", frame );

    return string( frameKey );
  }

  bool KchDetectionDb::setMeta( unsigned int length, int width, int height, float fps )
  {
    _fps = fps;
    _imageSize = Size( width, height );
    _vidLength = length;

    return saveMeta();
  }


  bool KchDetectionDb::saveMeta( void )
  {
    FileStorage fs("foo.yml", FileStorage::WRITE | FileStorage::MEMORY );

    write( fs, MetaFpsKey, _fps );
    write( fs, MetaWidthKey, _imageSize.width );
    write( fs, MetaHeightKey, _imageSize.height );
    write( fs,  MetaLengthKey,  _vidLength );

    return _db.set( MetaKey, fs.releaseAndGetString() );
  }

  void KchDetectionDb::loadMeta( void )
  {
    string metaStr;
    if( _db.get( MetaKey, &metaStr ) ) {
      FileStorage fs( metaStr, FileStorage::READ | FileStorage::MEMORY );

      fs[ MetaFpsKey ] >> _fps;
      float width, height;
      fs[ MetaWidthKey ] >> width;
      fs[ MetaHeightKey ] >> height;
      _imageSize = Size( width, height );
      fs[ MetaLengthKey ] >> _vidLength;


	// TODO:   Figure out why vidLength is ocassionally incorrect
	cerr << "Meta tag says this db has " << _vidLength << " entries.";
	_vidLength = maxKey();
	cerr << "But the max key is " << _vidLength;

    }
  }
}
