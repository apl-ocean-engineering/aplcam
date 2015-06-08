#ifndef __CALIBRATION_OPTS_COMMON_H__
#define __CALIBRATION_OPTS_COMMON_H__

#include <opencv2/calib3d.hpp>
#include <string>

#include "image.h"
#include "file_utils.h"

#include "distortion_model.h"

namespace AplCam {

  using std::string;

  using Distortion::DistortionModel;


  class CalibrationOptsCommon {
    public:


    CalibrationOptsCommon()
      : dataDir("../data"),
      boardName(), cameraName(), calibrationFile(),
      calibType( DistortionModel::CALIBRATION_NONE ),
      huberLoss( false ),
      fixSkew( true )
    {;}

    string dataDir;
    string boardName;
    string cameraName;
    string calibrationFile;
    DistortionModel::CalibrationType_t calibType;

    bool huberLoss, fixSkew;

    const string boardPath( void ) const
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string cachePath( void ) const
    { return dataDir + "/cache"; }

    const string imageCache( const Image &image ) const
    { return cachePath() + "/" + image.hash() + ".yml"; }

    const string tmpPath( const string &file ) const
    { return dataDir + "/tmp/" + file; }

    const string cameraPath( const string &filename ) const
    {
      string camDir(  dataDir + "/cameras/" + cameraName + "/" );
      if( !directory_exists( camDir ) ) mkdir_p( camDir );
      return camDir + filename;
    }

    virtual int calibFlags( void ) const {
      int flags = 0;

      if( huberLoss ) { flags |= Distortion::CALIB_HUBER_LOSS; }

      // FIX_SKEW disappeared from OpenCV...
      //if( fixSkew )   { flags |= cv::CALIB_FIX_SKEW; }

      return flags;
    }

    virtual bool validate( string &msg)
    {
      if( boardName.empty() ) { msg = "Board name not set"; return false; }
      if( cameraName.empty() ) { msg = "Camea name not set"; return false; }
      if( calibType == DistortionModel::CALIBRATION_NONE ) { msg = "Calibration type not specified"; return false; }

      if( calibrationFile.empty() ) calibrationFile = cameraPath( mkCameraFileName() );

      return true;
    }
      string mkCameraFileName( void ) const
    {
      char strtime[32], buffer[80];
      time_t tt;
      time( &tt );
      struct tm *t2 = localtime( &tt );
      strftime( strtime, 32, "%y%m%d_%H%M%S", t2 );
      snprintf( buffer, 79, "%s_%s.yml", cameraName.c_str(), strtime );
      return  string( buffer );
    }



  };



}

#endif
