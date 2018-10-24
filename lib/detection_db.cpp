
#include <fstream>
#include <iomanip>

#include "AplCam/detection_db.h"


namespace AplCam {

  using namespace std;

  InMemoryDetectionDb::InMemoryDetectionDb(  )
    : _filename("")
  {
  }

  InMemoryDetectionDb::InMemoryDetectionDb( const std::string &filename )
    : _filename(filename)
  {
    if( !filename.empty() ) {
      load();
    }
  }

  InMemoryDetectionDb::~InMemoryDetectionDb()
  {
    save();
  }

  void InMemoryDetectionDb::setFilename( const std::string &filename ) {
    _filename = filename;
  }

  void InMemoryDetectionDb::save()
  {
    json j = *this;

    ofstream out( _filename );

    // From the json.hpp docs: the setw manipulator was overloaded to set the indentation for pretty printing
    out << std::setw(4) << j;
  }

  void InMemoryDetectionDb::load()
  {
    ifstream in( _filename );
    json j;
    in >> j;

    *this = j;
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

  void from_json(const json& j, InMemoryDetectionDb& db) {
    std::cerr << "Loading from JSON!" << std::endl;

    db._map.clear();

    json jdet = j["detections"];

    for (json::iterator det = jdet.begin(); det != jdet.end(); ++det) {
        std::cerr << "   loading: " << det.key() << std::endl;

        std::shared_ptr<Detection> detection( new Detection );
        *(detection.get()) = det.value();

        db.insert( det.key(), detection );
    }

  }


}
