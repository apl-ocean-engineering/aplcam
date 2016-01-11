#ifndef __HOUGH_CIRCLES_H__
#define __HOUGH_CIRCLES_H__

#include <opencv2/core.hpp>
#include <vector>

namespace AplCam {

using cv::Mat;

typedef std::vector< cv::Vec3d > Circles_t;

void HoughCircles( Mat &src_image, Circles_t &output,
                  int method, double dp, double min_dist,
                  double param1, double param2,
                  int min_radius, int max_radius );

}

#endif
