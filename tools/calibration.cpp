#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

#include "my_undistort.h"

#include "distortion_model.h"
using namespace Distortion;

#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "image.h"

using namespace cv;
using namespace std;

class CalibrationOpts {
  public:
    CalibrationOpts()
      : dataDir("data"),
      boardName(), 
      inFiles(),
      ignoreCache( false )
  {;}

    bool validate( string &msg)
    {
      if( boardName.empty() ) { msg = "Board name not set"; return false; }
      if( cameraName.empty() ) { msg = "Camea name not set"; return false; }

      return true;
    }

    string dataDir;
    string boardName;
    string cameraName;
    vector< string > inFiles;
    bool ignoreCache, retryUnregistered;

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



    //== Option parsing and help ==
    void help()
    {
      printf( "This is a camera calibration sample.\n"
          "Usage: calibration\n"
          "     -d <data directory>      # Specify top-level directory for board/camera/cache files.\n"
          "     --board,-b <board_name>    # Name of calibration pattern\n"
          "     --camera, -c <camera_name> # Name of camera\n"
          "     --ignore-cache, -i       # Ignore and overwrite files in cache\n"
          "     --retry-unregistered, -r   # Re-try to find the chessboard if the cache file is empty\n"
          //     "     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
          //     "                              # (used only for video capturing)\n"
          //     "     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
          //     "     [-op]                    # write detected feature points\n"
          //     "     [-oe]                    # write extrinsic parameters\n"
          //     "     [-zt]                    # assume zero tangential distortion\n"
          //     "     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
          //     "     [-p]                     # fix the principal point at the center\n"
          //     "     [-v]                     # flip the captured images around the horizontal axis\n"
          //     "     [-V]                     # use a video file, and not an image list, uses\n"
          //     "                              # [input_data] string for the video file name\n"
          //     "     [-su]                    # show undistorted images after calibration\n"
          "     [input_data]             # list of files to use\n"
          "\n" );
      //printf("\n%s",usage);
      //printf( "\n%s", liveCaptureHelp );
    }


    void parseOpts( int argc, char **argv )
    {
      static struct option long_options[] = {
        { "data_directory", true, NULL, 'd' },
        { "board", true, NULL, 'b' },
        { "camera", true, NULL, 'c' },
        { "ignore-cache", false, NULL, 'i' },
        { "retry-unregistered", false, NULL, 'r' },
        { "help", false, NULL, '?' },
        { 0, 0, 0, 0 }
      };


      if( argc < 2 )
      {
        help();
        exit(1);
      }

      int indexPtr;
      int optVal;
      while( (optVal = getopt_long( argc, argv, "irb:c:d:?", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'd':
            dataDir = optarg;
            break;
          case 'b':
            boardName = optarg;
            break;
          case 'c':
            cameraName = optarg;
            break;
          case 'i':
            ignoreCache = true;
            break;
          case 'r':
            retryUnregistered = true;
            break;
          case '?': 
            help();
            break;
          default:
            exit(-1);

        }
      }

      if( optind == argc )
      {
        cout << "No input files specified." << endl;
        exit(-1);
      }

      for( int i = optind; i < argc; ++i ) {
        string infile( argv[i] );

        if( !file_exists( infile ) ) {
          cout << "Couldn't open input file \"" << infile << "\"" << endl;
          exit(-1);
        }

        inFiles.push_back( infile );
      }

      //    for( i = 1; i < argc; i++ )
      //    {
      //        const char* s = argv[i];
      //        if( strcmp( s, "-w" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%u", &boardSize.width ) != 1 || boardSize.width <= 0 )
      //                return fprintf( stderr, "Invalid board width\n" ), -1;
      //        }
      //        else if( strcmp( s, "-h" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%u", &boardSize.height ) != 1 || boardSize.height <= 0 )
      //                return fprintf( stderr, "Invalid board height\n" ), -1;
      //        }
      //        else if( strcmp( s, "-pt" ) == 0 )
      //        {
      //            i++;
      //            if( !strcmp( argv[i], "circles" ) )
      //                pattern = CIRCLES_GRID;
      //            else if( !strcmp( argv[i], "acircles" ) )
      //                pattern = ASYMMETRIC_CIRCLES_GRID;
      //            else if( !strcmp( argv[i], "chessboard" ) )
      //                pattern = CHESSBOARD;
      //            else
      //                return fprintf( stderr, "Invalid pattern type: must be chessboard or circles\n" ), -1;
      //        }
      //        else if( strcmp( s, "-s" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%f", &squareSize ) != 1 || squareSize <= 0 )
      //                return fprintf( stderr, "Invalid board square width\n" ), -1;
      //        }
      //        ..else if( strcmp( s, "-n" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%u", &nframes ) != 1 || nframes <= 3 )
      //                return printf("Invalid number of images\n" ), -1;
      //        }
      //        else if( strcmp( s, "-a" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%f", &aspectRatio ) != 1 || aspectRatio <= 0 )
      //                return printf("Invalid aspect ratio\n" ), -1;
      //            flags |= CV_CALIB_FIX_ASPECT_RATIO;
      //        }
      //        else if( strcmp( s, "-d" ) == 0 )
      //        {
      //            if( sscanf( argv[++i], "%u", &delay ) != 1 || delay <= 0 )
      //                return printf("Invalid delay\n" ), -1;
      //        }
      //        else if( strcmp( s, "-op" ) == 0 )
      //        {
      //            writePoints = true;
      //        }
      //        else if( strcmp( s, "-oe" ) == 0 )
      //        {
      //            writeExtrinsics = true;
      //        }
      //        else if( strcmp( s, "-zt" ) == 0 )
      //        {
      //            flags |= CV_CALIB_ZERO_TANGENT_DIST;
      //        }
      //        else if( strcmp( s, "-p" ) == 0 )
      //        {
      //            flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
      //        }
      //        else if( strcmp( s, "-v" ) == 0 )
      //        {
      //            flipVertical = true;
      //        }
      //        else if( strcmp( s, "-V" ) == 0 )
      //        {
      //            videofile = true;
      //        }
      //        else if( strcmp( s, "-o" ) == 0 )
      //        {
      //            outputFilename = argv[++i];
      //        }
      //        else if( strcmp( s, "-su" ) == 0 )
      //        {
      //            showUndistorted = true;
      //        }
      //        else if( s[0] != '-' )
      //        {
      //            if( isdigit(s[0]) )
      //                sscanf(s, "%d", &cameraId);
      //            else
      //                inputFilename = s;
      //        }
      //        else
      //            return fprintf( stderr, "Unknown option %s", s ), -1;
      //    }

      string msg;
      if( !validate( msg ) ) {
        cout << "Error: " <<  msg << endl;
        exit(-1);
      }
    }


};


static double computeReprojectionErrors(
    const vector<vector<Point3d> >& objectPoints,
    const vector<vector<Point2d> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors )
{
  vector<Point2d> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for( i = 0; i < (int)objectPoints.size(); i++ )
  {
    if( objectPoints[i].size() > 0 ) {
      //fisheye::projectPoints( Mat( objectPoints[i] ), imagePoints2, rvecs[i], tvecs[i],
      //    cameraMatrix, distCoeffs );
      projectPoints( Mat( objectPoints[i] ), rvecs[i], tvecs[i],
          cameraMatrix, distCoeffs, imagePoints2);

      err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
      int n = (int)objectPoints[i].size();
      perViewErrors[i] = (float)std::sqrt(err*err/n);
      totalErr += err*err;
      totalPoints += n;
    }
  }

  return std::sqrt(totalErr/totalPoints);
}

static bool runCalibration( vector<vector<Point2d> > imagePoints,
    vector< vector<Point3d> > objectPoints,
    Size imageSize, 
    float aspectRatio,
    int flags, Mat& cameraMatrix, Mat& distCoeffs,
    vector< Vec3d > &rvecs, vector< Vec3d > &tvecs,
    vector<float>& reprojErrs,
    double& totalAvgErr)
{
  //cameraMatrix = Mat::eye(3, 3, CV_64F);
  //if( flags & CV_CALIB_FIX_ASPECT_RATIO )
  //  cameraMatrix.at<double>(0,0) = aspectRatio;

  //distCoeffs = Mat::zeros(8, 1, CV_64F);
  //distCoeffs.create(0,0,CV_64F);

  // Call the functional form so we can get the RMS error
//  float fEstimate = max( imageSize.width, imageSize.height )/ CV_PI;
//  Matx33d kInitial( fEstimate, 0, imageSize.width/2.0 - 0.5,
//      0, fEstimate, imageSize.height/2.0 - 0.5,
//      0, 0, 1. );
  Fisheye fe;
  double rms = fe.calibrate( objectPoints, imagePoints, 
      imageSize, rvecs, tvecs, flags );

//  ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
  printf("RMS error reported by calibrateCamera: %g\n", rms);
//
//  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
//
 bool ok = true;
//
//  totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
//      rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
//
  return ok;
}


static void saveCameraParams( const string& filename,
    Size imageSize, const Board &board,
    const vector< Image > &imagesUsed,
    float aspectRatio, int flags,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    const vector<Vec3d>& rvecs, const vector<Vec3d>& tvecs,
    const vector<float>& reprojErrs,
    const vector<vector<Point2d> >& imagePoints,
    double totalAvgErr )
{

  FileStorage out( filename, FileStorage::WRITE );

  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  char buf[1024];
  strftime( buf, sizeof(buf)-1, "%c", t2 );

  out << "calibration_time" << buf;

  if( !rvecs.empty() || !reprojErrs.empty() )
    out << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
  out << "image_width" << imageSize.width;
  out << "image_height" << imageSize.height;
  out << "board_name" << board.name;
  out << "board_width" << board.size().width;
  out << "board_height" << board.size().height;
  out << "square_size" << board.squareSize;

  if( flags & CV_CALIB_FIX_ASPECT_RATIO )
    out << "aspectRatio" << aspectRatio;

  if( flags != 0 )
  {
    sprintf( buf, "flags: %s%s%s%s",
        flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
        flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
        flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
        flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
    cvWriteComment( *out, buf, 0 );
  }

  out << "flags" << flags;

  out << "camera_matrix" << cameraMatrix;
  out << "distortion_coefficients" << distCoeffs;

  out << "avg_reprojection_error" << totalAvgErr;
  if( !reprojErrs.empty() )
    out << "per_view_reprojection_errors" << Mat(reprojErrs);

//  if( !rvecs.empty() && !tvecs.empty() )
//  {
//    CV_Assert(rvecs[0].type() == tvecs[0].type());
//    Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
//    for( int i = 0; i < (int)rvecs.size(); i++ )
//    {
//      Mat r = bigmat(Range(i, i+1), Range(0,3));
//      Mat t = bigmat(Range(i, i+1), Range(3,6));
//
//      CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
//      CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
//      //*.t() is MatExpr (not Mat) so we can use assignment operator
//      r = rvecs[i].t();
//      t = tvecs[i].t();
//    }
//    cvWriteComment( *out, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
//    out   << "extrinsic_parameters" << bigmat;
//  }

  out << "images_used" << "[";
  for( vector<Image>::const_iterator img = imagesUsed.begin(); img < imagesUsed.end(); ++img ) {
    out << img->fileName();
  }
  out << "]";

  if( !imagePoints.empty() )
  {
    Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
    for( int i = 0; i < (int)imagePoints.size(); i++ )
    {
      Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
      Mat imgpti(imagePoints[i]);
      imgpti.copyTo(r);
    }
    out << "image_points" << imagePtMat;
  }
}


static string mkCameraFileName( void )
{
  char strtime[32];
  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  strftime( strtime, 32, "cal_%y%m%d_%H%M%S.yml", t2 );
  return  string( strtime );
}




int main( int argc, char** argv )
{

  CalibrationOpts opts;

  opts.parseOpts( argc, argv );

  Board *board = Board::load( opts.boardPath(), opts.boardName );

  Size imageSize;
  float aspectRatio = 1.f;
  Mat cameraMatrix, distCoeffs;
  bool writeExtrinsics = false, writePoints = false;

  vector<vector<Point2d> > imagePoints;
  vector<vector<Point3d> > objectPoints;

  if( opts.inFiles.size() < 1 ) {
    cout << "No input files specified on command line." << endl;
    exit(-1);
  }

  vector<Image> imagesUsed;

  if( opts.ignoreCache ) cout << "Ignoring cached data." << endl;
  for( int i = 0; i < opts.inFiles.size(); ++i ) {
    cout << "Processing " << i << " : " << opts.inFiles[i] << endl;
    Mat view, viewGray;

    view = imread(opts.inFiles[i], 1);

    imageSize = view.size();

    //if(!view.data)
    //{
    //  if( imagePoints.size() > 0 )
    //    runAndSave(outputFilename, imagePoints, imageSize,
    //        boardSize, pattern, squareSize, aspectRatio,
    //        flags, cameraMatrix, distCoeffs,
    //        writeExtrinsics, writePoints);
    //  break;
    //}

    Image img( opts.inFiles[i], view );
    Detection *detection = NULL;

    // Check for cached data
    string detectionCacheFile = opts.imageCache( img );
    bool doRegister = true;

    if( !opts.ignoreCache && (detection = Detection::loadCache( detectionCacheFile )) != NULL ) {
      doRegister = false;
      if( opts.retryUnregistered && detection && (detection->points.size() == 0) ) doRegister = true;
    }

    if( doRegister == false ) {
      cout << "  ... loaded data from cache." << endl;
    } else {

      cout << "  No cached data, searching for calibration pattern." << endl;

      //if( flipVertical )
      //  flip( view, view, 0 );

      vector<Point2f> pointbuf;
      cvtColor(view, viewGray, COLOR_BGR2GRAY);

      detection = board->detectPattern( viewGray, pointbuf );

      if( detection->found )  
        cout << "  Found calibration pattern." << endl;

      detection->writeCache( *board, detectionCacheFile );
    }

    if( detection->points.size() > 3 ) {
      imagesUsed.push_back( img );

// Whoops.  Type conversion from vec<Point2f> to vec<Point2d> that needs to be cleaned up later
vector< Point2d > imgPts( detection->points.size() );
std::copy( detection->points.begin(), detection->points.end(), imgPts.begin() );

vector< Point3d > wldPts( detection->corners.size() );
std::copy( detection->corners.begin(), detection->corners.end(), wldPts.begin() );

      imagePoints.push_back( imgPts );
      objectPoints.push_back( wldPts );

      detection->drawCorners(  *board, view );
    }

    string outfile( opts.tmpPath( img.basename() ) );
    mkdir_p( outfile );
    imwrite(  outfile, view );

    delete detection;
  }


  cout << "Using points from " << imagePoints.size() << "/" << opts.inFiles.size() << " images" << endl;

  string cameraFile( opts.cameraPath(mkCameraFileName() ) );
  vector< Vec3d > rvecs, tvecs;
  vector<float> reprojErrs;
  double totalAvgErr = 0;

  int flags = CV_CALIB_ZERO_TANGENT_DIST;

  bool ok = runCalibration(imagePoints, objectPoints,
      imageSize, aspectRatio, flags, cameraMatrix, distCoeffs,
      rvecs, tvecs, reprojErrs, totalAvgErr);

//  _if( ok ) {
//  saveCameraParams( cameraFile, imageSize,
//      *board, imagesUsed, aspectRatio,
//      flags, cameraMatrix, distCoeffs,
//      writeExtrinsics ? rvecs : vector<Mat>(),
//      writeExtrinsics ? tvecs : vector<Mat>(),
//      writeExtrinsics ? reprojErrs : vector<float>(),
//      writePoints ? imagePoints : vector<vector<Point2d> >(),
//      totalAvgErr );
//  }
//
//
//
//
//  // Redraw each image with rectified points
//  double alpha = 1;   // As a reminder, alpha = 0 means all pixels in undistorted image are correct
//                      //                alpha = 1 means all source image pixels are included
//                      //
//  Mat optimalCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, alpha, 
//        Size(), NULL );
//  //optimalCameraMatrix = cameraMatrix;
//  
//  cout << "Distortion coefficients: " << endl << distCoeffs << endl;
//  cout << "Calculated camera matrix: " << endl << cameraMatrix << endl;
//  cout << "Optimal camera matrix: " << endl << optimalCameraMatrix << endl;
//
//  Mat map1, map2;
//  initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat::eye(3,3,CV_64F), optimalCameraMatrix,
//      imageSize, CV_16SC2, map1, map2);
//  //fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat::eye(3,3,CV_64F), cameraMatrix,
//  //    imageSize, CV_16SC2, map1, map2);
//
//  for( int i = 0; i < imagesUsed.size(); ++i ) {
//
//    string outfile;
//    if( objectPoints[i].size() > 0 ) {
//      vector<Point2d> imagePoints2;
//      Mat out;
//      //fisheye::projectPoints(Mat(objectPoints[i]), imagePoints2, rvecs[i], tvecs[i],
//       //   cameraMatrix, distCoeffs);
//      projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
//          cameraMatrix, distCoeffs, imagePoints2);
//
//      imagesUsed[i].img().copyTo( out );
//      for( int j = 0; j < imagePoints[i].size(); ++j ) {
//        circle( out, imagePoints[i][j], 5, Scalar(0,0,255), 1 );
//
//        circle( out, imagePoints2[j], 5, Scalar(0,255,0), 1 );
//      }
//
//
//      outfile  = opts.tmpPath( String("reprojection/") +  imagesUsed[i].basename() );
//      mkdir_p( outfile );
//      imwrite(  outfile, out );
//    }
//   
//    Mat rview;
//    remap( imagesUsed[i].img(), rview, map1, map2, INTER_LINEAR);
//
//    const int N = 9;
//    int x, y, k;
//    vector< Point2d > pts, undPts;
//
//    for( y = k = 0; y < N; y++ )
//      for( x = 0; x < N; x++ )
//        pts.push_back( Point2d((float)x*imageSize.width/(N-1),
//                           (float)y*imageSize.height/(N-1)) );
//
//    myUndistortPoints(pts, undPts, cameraMatrix, distCoeffs, noArray(), optimalCameraMatrix );
//
//    for( y = k = 0; y < N; y++ )
//      for( x = 0; x < N; x++ ) {
////        cout << pts[k] <<  "  " << undPts[k] << endl;
//        Point2d thisPt( undPts[k++] );
//        if( thisPt.x >=0 && thisPt.x <= rview.size().width &&
//            thisPt.y >=0 && thisPt.y <= rview.size().height )
//          circle( rview, thisPt, 5, Scalar(0,255,255), 2 );
//      }
//
//
//    outfile = opts.tmpPath( String("undistorted/") +  imagesUsed[i].basename() );
//    mkdir_p( outfile );
//    imwrite(  outfile, rview );
//
//  }


  //    if( inputFilename )
  //    {
  //        if( !videofile && readStringList(inputFilename, imageList) )
  //            mode = CAPTURING;
  //        else
  //            capture.open(inputFilename);
  //    }
  //    else
  //        capture.open(cameraId);
  //
  //    if( !capture.isOpened() && imageList.empty() )
  //        return fprintf( stderr, "Could not initialize video (%d) capture\n",cameraId ), -2;
  //
  //    if( !imageList.empty() )
  //        nframes = (int)imageList.size();
  //
  //    if( capture.isOpened() )
  //        printf( "%s", liveCaptureHelp );
  //
  //    namedWindow( "Image View", 1 );
  //
  //    for(i = 0;;i++)
  //    {
  //
  //    if( !capture.isOpened() && showUndistorted )
  //    {
  //        Mat view, rview, map1, map2;
  //        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
  //                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
  //                                imageSize, CV_16SC2, map1, map2);
  //
  //        for( i = 0; i < (int)imageList.size(); i++ )
  //        {
  //            view = imread(imageList[i], 1);
  //            if(!view.data)
  //                continue;
  //            //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
  //            remap(view, rview, map1, map2, INTER_LINEAR);
  //            imshow("Image View", rview);
  //            int c = 0xff & waitKey();
  //            if( (c & 255) == 27 || c == 'q' || c == 'Q' )
  //                break;
  //        }
  //    }
  //

  // Put this after outputting the undistorted images.  Why?  Do get it after all the zlib
  // error messages
  printf("%s. avg reprojection error = %.2f\n",
      ok ? "Calibration succeeded" : "Calibration failed",
      totalAvgErr);
    cout << "Writing results to " << cameraFile << endl;

  delete board;

  return 0;
}