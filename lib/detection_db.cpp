
#include <fstream>
#include <iomanip>

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

    // From the json.hpp docs: the setw manipulator was overloaded to set the indentation for pretty printing
    out << std::setw(4) << j;
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
    j = {};

    json detections = {};

    for( auto const &itr : p._map ) {
      detections[itr.first] = *(itr.second);
    }

    j["detections"] = detections;
  }

  void from_json(const json& j, InMemoryDetectionDb& p) {

  }


}
