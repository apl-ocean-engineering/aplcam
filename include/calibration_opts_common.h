#ifndef __CALIBRATION_OPTS_COMMON_H__
#define __CALIBRATION_OPTS_COMMON_H__

#include <string>

namespace AplCam {

  using std::string;


  class CalibrationOptsCommon {
    public:

    typedef enum { ANGULAR_POLYNOMIAL, RADIAL_POLYNOMIAL } CalibrationType_t;

    CalibrationOptsCommon()
      : dataDir("data"),
      boardName(), cameraName(), calibrationFile(),
      calibFlags(0),
      calibType( ANGULAR_POLYNOMIAL )
    {;}

    string dataDir;
    string boardName;
    string cameraName;
    string calibrationFile;
    int calibFlags;
    CalibrationType_t calibType;

    const string boardPath( void )
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string cachePath( void )
    { return dataDir + "/cache"; }

    const string imageCache( const Image &image )
    { return cachePath() + "/" + image.hash() + ".yml"; }

    const string tmpPath( const string &file )
    { return dataDir + "/tmp/" + file; }

    const string cameraPath( const string &filename )
    {
      string camDir(  dataDir + "/cameras/" + cameraName + "/" );
      if( !directory_exists( camDir ) ) mkdir_p( camDir );
      return camDir + filename;
    }



    virtual bool validate( string &msg)
    {
      if( boardName.empty() ) { msg = "Board name not set"; return false; }
      if( cameraName.empty() ) { msg = "Camea name not set"; return false; }

      if( calibrationFile.empty() ) calibrationFile = cameraPath( mkCameraFileName() );

      return true;
    }

    string mkCameraFileName( void )
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