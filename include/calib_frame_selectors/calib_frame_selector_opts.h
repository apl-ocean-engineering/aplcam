
#ifndef __CALIB_FRAME_SELECTOR_OPTS_H__
#define __CALIB_FRAME_SELECTOR_OPTS_H__

#include <limits.h>
#include <unistd.h>
#include <getopt.h>

#include <string>

namespace AplCam {

  namespace CalibFrameSelectors {

    using std::string;

    struct RandomSelectorOpts {
      RandomSelectorOpts()
        : count(-1) {;}

      long int count;

      static struct option long_options[]; 
      bool parseOpts( int argc, char **argv, string &msg );
    };

    struct IntervalSelectorOpts {
      IntervalSelectorOpts()
        : start(0), end(INT_MAX), interval(1) {;}

      int start, end, interval;

      static struct option long_options[];
      bool parseOpts( int argc, char **argv, string &msg );
    };

    struct KeyframeSelectorOpts {
      KeyframeSelectorOpts()
        : minOverlap( 0.2 ) {;}

      float minOverlap;

      static struct option long_options[];
      bool parseOpts( int argc, char **argv, string &msg );
    };



  }
}

#endif
