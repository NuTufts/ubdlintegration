#include "PyNet_CosmicMRCNN.h"

#include <sstream>

#include "larcv/core/DataFormat/SparseImage.h"
#include "larcv/core/json/json_utils.h"
#include "larcv/core/PyUtil/PyUtils.h"

namespace ubcv {
namespace ubdlintegration {

  PyNetCosmicMRCNN::PyNetCosmicMRCNN( const std::vector< std::string >& weight_file_v,
				      const std::vector< std::string >& config_file_v ) 
    : larcv::larcv_base("PyNetCosmicMRCNN"),
      pModule(nullptr),
      pFunc(nullptr)
  {
    set_verbosity(larcv::msg::kDEBUG);

    std::cout << "[PyNetCosmicMRCNN] py initialize" << std::endl;
    Py_Initialize();

    std::cout << "[PyNetCosmicMRCNN] import numpy array" << std::endl;
    larcv::SetPyUtil();

    LARCV_INFO() << "[PyNetCosmicMRCNN] import script: inference_mrcnn" << std::endl;      
    PyObject *pName   = PyUnicode_FromString("inference_mrcnn");
    try {
      pModule = PyImport_Import(pName);
    }
    catch ( std::exception& e ) {
      std::stringstream oops;
      oops << "exception caught attempting to load inference_mrcnn module" << std::endl
	   << " err: " << e.what() << std::endl
	   << " pyerr: " << std::endl;
      LARCV_NORMAL() << oops.str() << std::endl;
      PyErr_Print();
      LARCV_CRITICAL() << oops.str();
    }

    if ( !pModule ) {
      std::string msg = "[PyNetCosmicMRCNN] inference_mrcnn could not be loaded";
      LARCV_NORMAL() << "=====================================" << std::endl;
      LARCV_NORMAL() << msg << std::endl;
      PyErr_Print();
      LARCV_DEBUG() << "python path directories" << std::endl;
      char pathname[10];
      sprintf(pathname,"path");
      PyObject* path = PySys_GetObject(pathname);
      int nfolders = PyList_Size(path);
      for ( int i=0; i<nfolders; i++ ) {
	LARCV_DEBUG() << PyString_AsString( PyList_GetItem(path,(Py_ssize_t)i) ) << std::endl;
      }
      LARCV_CRITICAL() << msg << std::endl;
      throw std::runtime_error(msg);
    }
    std::cout << "[PyNetCosmicMRCNN] import module: " << pModule << std::endl;
    pFunc   = PyObject_GetAttrString(pModule,"forwardpass");
    std::cout << "[PyNetCosmicMRCNN] got function 'forwardpass': " << pFunc << std::endl;
    Py_DECREF(pName);

    // weight files
    _weight_file_v.clear();
    for ( auto const& weightfile : weight_file_v )
      _weight_file_v.push_back( weightfile );

    // config files
    _config_file_v.clear();
    for ( auto const& configfile : config_file_v )
      _config_file_v.push_back( configfile );

  }

  PyNetCosmicMRCNN::~PyNetCosmicMRCNN() {

    std::cout << "[PyNetCosmicMRCNN] python finalize" << std::endl;
    Py_Finalize();
    
  }
  
  int PyNetCosmicMRCNN::run_cosmic_mrcnn( const std::vector<larcv::Image2D>& wholeview_v, 
					  const int run, const int subrun, const int event, 
					  const float threshold,
					  std::vector< std::vector<larcv::ClusterMask> >& output_vv,
					  const std::string script_path ) {
    

    for ( auto const& img : wholeview_v ) {

      std::cout << "[PyNetCosmicMRCNN] image plane: " << img.meta().plane() << std::endl;
      
      PyObject* bson = larcv::json::as_pybytes( img, run, subrun, event, 0 );

      PyObject *pWeightpath   = PyUnicode_FromString( _weight_file_v.at( (int)img.meta().plane() ).c_str() );
      PyObject *pConfigpath   = PyUnicode_FromString( _config_file_v.at( (int)img.meta().plane() ).c_str() );
      
      std::cout << "call function: " << pFunc << " bson=" << bson <<" weight=" << pWeightpath << " config=" << pConfigpath << std::endl;
      PyObject *pReturn = PyObject_CallFunctionObjArgs(pFunc,bson,pWeightpath,pConfigpath,NULL);
      std::cout << "python returned: " << pReturn << std::endl;

      if (!PyList_Check(pReturn)) {
	throw std::runtime_error("Return from pynet_deploy.inference_mrcnn.forwardpass was no a list");
      }
	
      auto nmasks = PyList_Size(pReturn);
      
      std::cout << "Number of masks returned from python: " << nmasks << std::endl;
      std::vector<larcv::ClusterMask> masks_v;
      for (int imask=0; imask<(int)nmasks; imask++ ) {
	PyObject* maskbson = PyList_GetItem(pReturn,(Py_ssize_t)imask);
	int ret_run, ret_subrun, ret_event, ret_id;

	larcv::ClusterMask cmask 
	  = larcv::json::clustermask_from_pybytes( maskbson, ret_run, ret_subrun, ret_event, ret_id );
						   
	masks_v.emplace_back( std::move(cmask) );
      }
      
      output_vv.emplace_back( std::move(masks_v) );
      
      std::cout << "dereference string arguments/paths" << std::endl;
      Py_DECREF(pWeightpath);
      Py_DECREF(pConfigpath);
      Py_DECREF(pReturn);
      
    }

    std::cout << "done calling python script" << std::endl;
    return 0;
  }
  

}
}
