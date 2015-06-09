#ifndef __CALIB_FRAME_SELECTOR_H__
#define __CALIB_FRAME_SELECTOR_H__

#include <string>
#include <sstream>

#include <kchashdb.h>

#include "bits_to_hex.h"
#include "random.h"
#include "detection_set.h"


namespace AplCam {

  namespace CalibFrameSelectors {

    using std::string;
    using std::stringstream;
    using kyotocabinet::DB;

    class FrameSelector {
      public:
        FrameSelector()
        {;}

        virtual ~FrameSelector()
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


        //RandomFrameSelector( const RandomSelectorOpts &opts ) 
        //    : _count( opts.count )
        //{;}


        virtual void generate( DetectionDb &db, DetectionSet &set );

      protected:

        long int _count;
    };

    class IntervalFrameSelector : public FrameSelector {
      public:
        IntervalFrameSelector( int s, int i, int e = INT_MAX )
          : _start( s ), _end( e ), _interval( i )
        {;}

        //  IntervalFrameSelector( const IntervalSelectorOpts &opts )
        //      : _start( opts.start ), _end(opts.end), _interval( opts.interval )
        //  {;}

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

    class KeyframeFrameSelector : public FrameSelector {
      public:
        KeyframeFrameSelector( const Board &board )
          : _board(board), _minOverlap( 0.3 )
        {;}

        //  KeyframeFrameSelector( const Board &board, const KeyframeSelectorOpts &opts )
        //      : _board( board ),_minOverlap( opts.minOverlap )
        //  {;}

        virtual void generate( DetectionDb &db, DetectionSet &set );

      protected:

        const Board &_board;
        float _minOverlap;

    };




  }
}


#endif
