#ifndef __PyNet_CosmicMRCNN_h__
#define __PyNet_CosmicMRCNN_h__

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include "Python.h"
#include "numpy/arrayobject.h"

/**
 * this function calls a python script that runs mask-rcnn
 *
 */

#include <vector>
#include "larcv/core/Base/larcv_base.h"
#include "larcv/core/DataFormat/Image2D.h"
#include "larcv/core/DataFormat/ClusterMask.h"

namespace ubcv {
namespace ubdlintegration {

  class PyNetCosmicMRCNN : public larcv::larcv_base {
    
  public:
    PyNetCosmicMRCNN( const std::vector<std::string>& weight_file_v,
		      const std::vector<std::string>& config_file_v );
    virtual ~PyNetCosmicMRCNN();

    int run_cosmic_mrcnn( const std::vector<larcv::Image2D>& wholeview_v, 
			  const int run, const int subrun, const int event, 
			  const float threshold,
			  std::vector< std::vector<larcv::ClusterMask> >& output_vv,			    
			  const std::string script_path="" );

  protected:

    PyObject *pModule;
    PyObject *pFunc;
    std::vector<std::string> _weight_file_v;
    std::vector<std::string> _config_file_v;

  };

}
}
#endif
