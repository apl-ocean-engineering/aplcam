#ifndef __CALIBRATION_SERIALIZER_H__
#define __CALIBRATION_SERIALIZER_H__

#include <string>

#include "AplCam/distortion/distortion_model.h"
#include "AplCam/calibration_result.h"
#include "AplCam/board/board.h"


namespace AplCam {

  using std::string;
  using Distortion::Camera;

  class CalibrationSerializer {
    public:

      CalibrationSerializer( void )
        : _camera( NULL ), _results( NULL ), _board( NULL )
      {;}

      CalibrationSerializer &setCamera( Camera *const cam )
      { _camera = cam; return *this; }

      CalibrationSerializer &setResult( CalibrationResult *const res )
      { _results = res; return *this; }

      CalibrationSerializer &setBoard( Board *const b )
      { _board = b; return *this; }

      string timeString( void ) const
      {
        time_t tt;
        time( &tt );
        struct tm *t2 = localtime( &tt );
        char buf[1024];
        strftime( buf, sizeof(buf)-1, "%c", t2 );
        return string(buf);
      }

      bool writeFile( const string &filename ) const
      {
        FileStorage fs( filename, FileStorage::WRITE );
        if( !fs.isOpened() ) return false;

        return serialize( fs );
      }

      virtual bool serialize( string &str ) const
      {
        FileStorage fs("foo.yml", FileStorage::WRITE | FileStorage::MEMORY );
        bool res = serialize( fs );
        str = fs.releaseAndGetString();
        return res;
      }

      bool serialize( FileStorage &fs ) const
      {
        fs << "calibration_time" << timeString();

        if( _board ) {
          fs << "board_name" << _board->name;
          fs << "board_width" << _board->size().width;
          fs << "board_height" << _board->size().height;
          fs << "square_size" << _board->squareSize;
        }
        if( _camera) _camera->write( fs );
        //if( _results ) _results->serialize( fs );

        return true;
      }

    protected:
      Camera *_camera;
      CalibrationResult *_results;
      Board *_board;

  };

}

#endif
