#include <stdlib.h>

#include "calib_frame_selectors/calib_frame_selector_opts.h"

namespace AplCam {
  namespace CalibFrameSelectors {

    //-- RandomSelectorOpts --
    struct option RandomSelectorOpts::long_options[] = {
      { "count", required_argument, NULL, 'c' },
      { 0, 0, 0, 0}
    };

    bool RandomSelectorOpts::parseOpts( int argc, char **argv, string &msg )
    {
      char optVal;
      int indexPtr;
      while( (optVal = getopt_long( argc, argv, "c:", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'c':
            count = atol( optarg );
            break;
          default:
            return false;
        }
      }

      if( count < 0 ) {
        msg = "Must specify count with -c option";
        return false;
      }

      return true;
    }


    //-- IntervalSelectorOpts --
    struct option IntervalSelectorOpts::long_options[] = {
      { "start", required_argument, NULL, 's' },
      { "end", required_argument, NULL, 'e' },
      { "interval", required_argument, NULL, 'i' },
      { 0, 0, 0, 0}
    };

    bool IntervalSelectorOpts::parseOpts( int argc, char **argv, string &msg )
    {
      char optVal;
      int indexPtr;
      while( (optVal = getopt_long( argc, argv, "e:i:s:", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'i':
            interval = atoi( optarg );
            break;
          case 's':
            start = atoi( optarg );
            break;
          case 'e':
            end = atoi( optarg );
            break;
          default:
            return false;
        }
      }

      if( interval <= 0 || start < 0 ) {
        msg = "Invalid interval or offset specified";
        return false;
      }

      return true;
    }

    //-- KeyframeSelectorOpts --
    struct option KeyframeSelectorOpts::long_options[] = {
      { "min-overlap", required_argument, NULL, 'm' },
      { 0, 0, 0, 0}
    };

    bool KeyframeSelectorOpts::parseOpts( int argc, char **argv, string &msg )
    {
      char optVal;
      int indexPtr;
      while( (optVal = getopt_long( argc, argv, "m:", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'm':
            minOverlap = atof( optarg );
            break;
          default:
            return false;
        }
      }

      if( minOverlap < 0.0 || minOverlap > 1.0 ) {
        msg = "Minimum overlap out of bounds";
        return false;
      }

      return true;
    }



  }
}
