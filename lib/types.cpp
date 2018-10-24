#include <opencv2/core/core.hpp>
#include "AplCam/types.h"

namespace AplCam {
  using namespace std;
  using namespace cv;

  static Vec4f TxReprojErrorsToVec( const ReprojError &error )
  {
    return Vec4f( error.projPoint[0], error.projPoint[1], error.error[0], error.error[1] );
  }

  void write(cv::FileStorage &fs, std::string a, const ReprojErrorVec& v )
  {
    // Lazy approach for now
    int sz = v.size();
    vector< Vec4f > out( sz );

    std::transform( v.begin(), v.end(), out.begin(), TxReprojErrorsToVec );

    cv::write( fs, a, Mat(out) );
  }

  void write(cv::FileStorage &fs, std::string a, const ReprojErrorVecVec &v )
  {
    vector< Vec4f > out;

    // Feeling lazy, this approach will be slower due to continual resizing of out..
    std::for_each( v.begin(), v.end(),
        [&]( const ReprojErrorVec &rpv ) {
          std::transform( rpv.begin(), rpv.end(), back_inserter( out ), TxReprojErrorsToVec );
        } );

    cv::write( fs, a, Mat(out) );
  }

}
