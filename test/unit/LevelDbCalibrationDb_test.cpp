
#include <iostream>

#include <gtest/gtest.h>

#include "AplCam/leveldb_calibration_db.h"

using namespace AplCam;
using namespace std;

namespace {

TEST( LevelDbCalibrationDb, Constructor ) {

  LevelDbCalibrationDb db("/tmp/calibration.ldb", true);

}


}
