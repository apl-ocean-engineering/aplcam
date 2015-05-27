
#include <opencv2/core/core.hpp>

#ifndef __TRENDNET_TIME_CODE_H__
#define __TRENDNET_TIME_CODE_H__

namespace TimeCode_1920x1080 {

const int timeCodeLeft   = 19;
const int timeCodeWidth  = 228;
const int timeCodeTop    =  8;
const int timeCodeHeight = 14;
const int timeCodeHalfHeight = timeCodeHeight / 2;

const cv::Rect timeCodeROI( timeCodeLeft, timeCodeTop, timeCodeWidth, timeCodeHeight );

const cv::Rect timeCodeAboveROI( timeCodeLeft, timeCodeTop - timeCodeHeight/2,
                                timeCodeWidth, timeCodeHeight/2 );

const cv::Rect timeCodeTopHalfROI( timeCodeLeft, timeCodeTop, 
                                timeCodeWidth, timeCodeHeight/2 );

const cv::Rect timeCodeBottomHalfROI( timeCodeLeft, timeCodeTop + timeCodeHeight/2,
                                timeCodeWidth, timeCodeHeight/2 );

const cv::Rect timeCodeBelowROI( timeCodeLeft, timeCodeTop + timeCodeHeight,
                                timeCodeWidth, timeCodeHeight/2 );

cv::Mat timeCodeMask();
cv::Mat timeCodeUnmask();
}



#endif
