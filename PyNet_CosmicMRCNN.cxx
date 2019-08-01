#include "PyNet_CosmicMRCNN.h"

#include "larcv/core/DataFormat/SparseImage.h"
#include "larcv/core/json/json_utils.h"
#include "larcv/core/PyUtil/PyUtils.h"

namespace ubcv {
namespace ubdlintegration {

  PyNetCosmicMRCNN::PyNetCosmicMRCNN() 
    : pModule(nullptr),
      pFunc(nullptr)
  {

    std::cout << "py initialize" << std::endl;
    Py_Initialize();

    std::cout << "import numpy array" << std::endl;
    //import_array1(0);
    //larcv::json::load_jsonutils();
    larcv::SetPyUtil();

    std::cout << "import script" << std::endl;      
    PyObject *pName   = PyUnicode_FromString("inference_mrcnn");
    pModule = PyImport_Import(pName);
    std::cout << "import module: " << pModule << std::endl;
    pFunc   = PyObject_GetAttrString(pModule,"forwardpass");
    std::cout << "got function: " << pFunc << std::endl;
    Py_DECREF(pName);

    // weight files
    _weight_dir = "/uboone/app/users/tmw/ups_dev/products/ubdl/v1_0_0/Linux64bit+3.10-2.17_e17_prof/ublarcvserver/app/ubmrcnn/weights/ubmrcnn_mcc8_v1";

  }

  PyNetCosmicMRCNN::~PyNetCosmicMRCNN() {

    std::cout << "python finalize" << std::endl;
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

      char config[50];
      sprintf( config, "pynet_deploy/mills_config_%d.yaml", (int)img.meta().plane() );
      
      char weight_path[512];
      sprintf( weight_path, "%s/mcc8_mrcnn_plane%d.pth", _weight_dir.c_str(), (int)img.meta().plane() );

      PyObject *pWeightpath   = PyUnicode_FromString(weight_path);
      PyObject *pConfigpath   = PyUnicode_FromString(config);
      
      std::cout << "call function: " << pFunc << " bson=" << bson <<" wieght=" << pWeightpath << " config=" << pConfigpath << std::endl;
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
	  = larcv::json::clustermask_from_pybytes( maskbson,
						   ret_run, ret_subrun, ret_event, ret_id );
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
