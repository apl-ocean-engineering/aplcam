
#include <iostream>

#include <gtest/gtest.h>

#include "AplCam/leveldb_detection_db.h"

using namespace AplCam;
using namespace std;

namespace {

TEST( LevelDbDetectionDb, Constructor ) {

  LevelDbDetectionDb db("/tmp/detection.ldb", true);

}


}
