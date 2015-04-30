#include "calib_frame_selectors/calib_frame_selectors.h"

namespace AplCam {

  namespace CalibFrameSelectors {

    void RandomFrameSelector::generate( DetectionDb &db, DetectionSet &set )
    {
      vector< string > keys;

      DB::Cursor *cur = db.cursor();
      cur->jump();

      string key;
      while( cur->get_key( &key, true) ) 
        if( isValidFrame( key ) ) keys.push_back(key);

      //          delete cur;

      long int c = std::min( _count, (long int)keys.size() );

      std::random_shuffle( keys.begin(), keys.end(), unaryRandom );
      keys.resize( c );

      for( vector< string >::iterator itr = keys.begin(); itr != keys.end(); ++itr ) {
        set.addDetection( db, stoi(*itr) );
      }

      stringstream strm;
      strm << "random(" << _count << ")_" << intsToHex( set.frames() );
      set.setName( strm.str() );
    }

  }
}


