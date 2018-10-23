#include "AplCam/calib_frame_selectors/calib_frame_selectors.h"

namespace AplCam {

  namespace CalibFrameSelectors {

    void RandomFrameSelector::generate( DetectionDb &db, DetectionSet &set )
    {
      // vector< int > keys;
      //
      // DB::Cursor *cur = db.cursor();
      // cur->jump();
      //
      // string key;
      // while( cur->get_key( &key, true) )
      //   if( isNotMeta( key ) ) {
      //     if( minTagCriteriaGiven() ) {
      //       int frame = stoi( key );
      //       Detection *detection = db.load( frame );
      //       if( hasMinTags( detection ) ) keys.push_back( frame );
      //     } else {
      //       keys.push_back( stoi( key ) );
      //     }
      //   }
      //
      // //          delete cur;
      //
      // long int c = std::min( _count, (long int)keys.size() );
      //
      // std::random_shuffle( keys.begin(), keys.end(), unaryRandom );
      // keys.resize( c );
      //
      // for( vector< int >::iterator itr = keys.begin(); itr != keys.end(); ++itr ) {
      //   set.addDetection( db, *itr );
      // }
      //
      // std::stringstream strm;
      // strm << "random(" << _count << ")_" << intsToHex( set.frames() );
      // set.setName( strm.str() );
    }

  }
}
