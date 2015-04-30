
#ifndef __APLCAM_RANDOM_H__
#define __APLCAM_RANDOM_H__

#include <random>

namespace AplCam {

  static std::minstd_rand0 _rand = std::minstd_rand0( time(NULL) );
  inline unsigned long unaryRandom( int i ) { return _rand() % i; }

}

#endif
