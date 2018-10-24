

#include "AplCam/calibration_result.h"


namespace AplCam {

  void   to_json(json& j, const Result& p) {
    p.to_json(j);
  }

  void from_json(const json& j, Result& p) {
    ;
  }

  void   to_json(json& j, const CalibrationResult& p) {
    p.to_json(j);
  }

  void from_json(const json& j, CalibrationResult& p) {
    ;
  }

}
