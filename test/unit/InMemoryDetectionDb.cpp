
#include <iostream>

#include <gtest/gtest.h>

#include "AplCam/detection_db.h"
#include "nlohmann/json.hpp"

namespace {

  using namespace AplCam;
  using namespace std;

  using nlohmann::json;

  TEST( InMemoryDetectionDb, Constructor ) {

    InMemoryDetectionDb db;

  }


  TEST( InMemoryDetectionDb, SerializeToJson ) {
    InMemoryDetectionDb db;

    json j = db;
  }

  TEST( InMemoryDetectionDb, SerializeFromJson ) {
    json j;

    InMemoryDetectionDb db = j;
  }

}
