#ifndef __CALIBRATION_SERIALIZER_H__
#define __CALIBRATION_SERIALIZER_H__

#include <string>

#include "AplCam/distortion/distortion_model.h"
#include "AplCam/calibration_result.h"
#include "AplCam/board/board.h"


namespace AplCam {

  using std::string;
  using Distortion::Camera;

  class JSONCalibrationSerializer : public CalibrationSerializer {
    public:

      JSONCalibrationSerializer( void )
        : CalibrationSerializer()
      {;}

      virtual bool serialize( string &str ) const
      {
        fs << "calibration_time" << timeString();

        if( _board ) {
          fs << "board_name" << _board->name;
          fs << "board_width" << _board->size().width;
          fs << "board_height" << _board->size().height;
          fs << "square_size" << _board->squareSize;
        }
        if( _camera) _camera->write( fs );
        if( _results ) _results->serialize( fs );

        return true;
      }

    protected:

  };

}

#endif
