
#include <fstream>

#include "AplCam/detection_db.h"


namespace AplCam {

  using namespace std;

  InMemoryDetectionDb::InMemoryDetectionDb( const std::string &filename )
    : _filename(filename)
  {
    if( !filename.empty() ) {
      //from_json()
    }
  }

  InMemoryDetectionDb::~InMemoryDetectionDb()
  {
    sync();
  }

  void InMemoryDetectionDb::sync()
  {
    json j = *this;

    ofstream out( _filename );
    out << j;
  }

  bool InMemoryDetectionDb::insert( const std::string &frame, const std::shared_ptr<Detection> &detection ) {
    _map.insert( std::make_pair( frame, detection ) );
    return true;
  }

  std::shared_ptr<Detection> InMemoryDetectionDb::at( const std::string &frame ) {
    return _map.at( frame );
  }

  bool InMemoryDetectionDb::setMeta( unsigned int length, int width, int height, float fps ) {
    return true;
  }

  //======

  void to_json(json& j, const InMemoryDetectionDb& p) {

  }

  void from_json(const json& j, InMemoryDetectionDb& p) {

  }


}
