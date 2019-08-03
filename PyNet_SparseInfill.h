#ifndef __PyNet_SparseInfill_h__
#define __PyNet_SparseInfill_h__

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include "Python.h"
#include "numpy/arrayobject.h"

/**
 * this function calls a python script that runs mask-rcnn
 *
 */

#include <vector>
#include "larcv/core/DataFormat/Image2D.h"
#include "larcv/core/DataFormat/ClusterMask.h"

namespace ubcv {
namespace ubdlintegration {

  class PyNetSparseInfill {
    
  public:
    PyNetSparseInfill( const std::vector<std::string>& weight_file_v );
    virtual ~PyNetSparseInfill();

    int run_sparse_cropped_infill( const std::vector< std::vector<larcv::SparseImage> >& cropped_vv, 
				   const int run, const int subrun, const int event,
				   std::vector<std::vector<larcv::SparseImage> >& results_vv,
				   const float threshold,
				   bool debug );

  protected:

    PyObject *pModule;
    PyObject *pFunc;
    std::vector<std::string> _weight_file_v;

  };

}
}
#endif
