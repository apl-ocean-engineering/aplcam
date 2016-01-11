
#include <iostream>
#include <sstream>
#include <glog/logging.h>

#include "AplCam/splitter_common.h"

using namespace std;
using namespace AplCam;

int main( int argc, char **argv )
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = true;
  FLAGS_minloglevel = 0;

  SplitterOpts opts;
  stringstream msg;
  if( !opts.parseArgs( argc, argv, msg ) ) {
    cout << msg.str() << endl;
    exit(-1);
  }

  SplitterApp main( opts );

  exit( main.run() ? 0 : -1 );

}
