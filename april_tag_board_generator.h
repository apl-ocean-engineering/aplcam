
#include <opencv2/core/core.hpp>
#include <vector>

#include <AprilTags/TagFamily.h>

#ifndef __APRIL_TAG_BOARD_GENERATOR_H__
#define __APRIL_TAG_BOARD_GENERATOR_H__

using cv::Point3f;
using cv::Point2f;
using cv::Mat;
using cv::Size;
using std::vector;

using AprilTags::TagFamily;

namespace cv {


  class AprilTagBoard
  {
    public:

      AprilTagBoard( const Size &arraySize = Size(8,6), const double tagSize = 1, 
          const double tagSpacing = 3 );

      Size arraySize( void ) const { return _arraySize; }
      Size boardSize( void ) const 
          { return Size( _tagSpacing * (arraySize().width+1),
                         _tagSpacing * (arraySize().height+1) ); }
     float boardAspectRatio( void ) const
          { return boardSize().height * 1.0 / boardSize().width; }

    private:

    Size _arraySize;
    double _tagSize;
    double _tagSpacing;

    TagFamily _tagFamily;
    Mat _tags;

  };


class AprilTagBoardGenerator 
{
public:
    double sensorWidth;
    double sensorHeight;


    AprilTagBoardGenerator(const AprilTagBoard &board );

    // Generates a randomized pose for the board then renders an image
    // of the board against the background bg.
    //
    // Returns the rendered image.
    //
    Mat generateImageOfBoard(const Mat& bg, const Mat& camMat, const Mat& distCoeffs, vector<Point2f>& corners) const;

private:

    // Calculate the field of view given the camera matrix, image size
    // and the sensor size (which is fixed)
    Point2f fieldOfView( const Mat &camMat, const Size &imgSize ) const;

    // Breaks a line segment into _segmentsPerEdge sub-segments
    void generateEdge(const Point3f& p1, const Point3f& p2, vector<Point3f>& out) const;

    vector<Point> generateContour( const vector<Point3f> &worldPts, 
        const Mat& camMat, const Mat& distCoeffs)  const;

    Mat drawBoard(const Mat& bg, const Mat& camMat, const Mat& distCoeffs,
        const Point3f& origin, const Point3f& pb1, const Point3f& pb2,
        const Size2f &boardSize,
        vector<Point2f>& corners) const;

    void generateBasis(Point3f& pb1, Point3f& pb2) const;

//    Point3f generateChessBoardCenter(const Mat& camMat, const Size& imgSize) const;


    const AprilTagBoard &_board;

    // Parameters for the randomized board generation
    static const double _minCos;
    static const double _cov;
    static const int _rendererResolutionMultiplier;

    static const size_t  _segmentsPerEdge;

    Mat rvec, tvec;
};

}

#endif
