

#ifndef __CALIB_FRAME_SELECTOR_H__
#define __CALIB_FRAME_SELECTOR_H__

#include <kchashdb.h>

#include "bits_to_hex.h"
#include "random.h"
#include "detection_set.h"

#include "calib_frame_selector_opts.h"


namespace AplCam {

  namespace CalibFrameSelectors {

    using kyotocabinet::DB;

    class FrameSelector {
      public:
        FrameSelector()
        {;}

        virtual void generate( DetectionDb &db, DetectionSet &set ) = 0;

        bool isValidFrame( const string &key )
        {
          return key != DetectionDb::MetaKey;
        }
    };


    class AllFrameSelector : public FrameSelector {
      public:
        AllFrameSelector( void ) {;}

        virtual void generate( DetectionDb &db, DetectionSet &set )
        {
          DB::Cursor *cur = db.cursor();
          cur->jump();
          string key;
          while( cur->get_key( &key, true ) ) 
            if( isValidFrame( key ) ) set.addDetection( db, stoi(key) );
          set.setName( "all" );

          //delete cur;

        }
    };

    class AllGoodFrameSelector : public FrameSelector {
      public:
        AllGoodFrameSelector( void ) {;}

        virtual void generate( DetectionDb &db, DetectionSet &set )
        {
          DB::Cursor *cur = db.cursor();
          cur->jump();
          string key, value;
          while( cur->get_key( &key, true ) ){
            if( !isValidFrame( key ) ) continue;

            int frame = stoi( key );
            Detection *detection = db.load( frame );
            if( detection ) {
              if( detection->rot[0] == 0.0 && detection->rot[1] == 0.0 && detection->rot[2] == 0.0 ) continue;

            set.addDetection( detection, frame );
            }

          }

          set.setName( "all" );

          //delete cur;

        }
    };



    class RandomFrameSelector : public FrameSelector {
      public:
        RandomFrameSelector( int c )
          : _count( c )
        {;}


        RandomFrameSelector( const RandomSelectorOpts &opts ) 
          : _count( opts.count )
        {;}


        virtual void generate( DetectionDb &db, DetectionSet &set )
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

      protected:

        long int _count;
    };

    class IntervalFrameSelector : public FrameSelector {
      public:
        IntervalFrameSelector( int s, int i, int e = INT_MAX )
          : _start( s ), _end( e ), _interval( i )
        {;}

        IntervalFrameSelector( const IntervalSelectorOpts &opts )
          : _start( opts.start ), _end(opts.end), _interval( opts.interval )
        {;}

        virtual void generate( DetectionDb &db, DetectionSet &set )
        {
          int e = std::min( _end, db.maxKey() );

          for( int i = _start; i < e; i += _interval ) 
            set.addDetection( db,  i );

          stringstream strm;
          strm << "interval(" << _start << "," << _interval << ',' << e << ")_" << intsToHex( set.frames() );
          set.setName( strm.str() );
        }

      protected:

        int _start, _end, _interval;

    };


  }
}


#endif
