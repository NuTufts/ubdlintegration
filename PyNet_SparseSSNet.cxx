#include "PyNet_SparseSSNet.h"

#include "larcv/core/DataFormat/SparseImage.h"
#include "larcv/core/json/json_utils.h"
#include "larcv/core/PyUtil/PyUtils.h"

namespace ubcv {
namespace ubdlintegration {

  PyNetSparseSSNet::PyNetSparseSSNet( const std::vector< std::string >& weight_file_v )
    : pModule(nullptr),
      pFunc(nullptr)
  {

    std::cout << "[PyNetSparseSSNet] py initialize" << std::endl;
    Py_Initialize();

    std::cout << "[PyNetSparseSSNet] import numpy array" << std::endl;
    larcv::SetPyUtil();

    std::cout << "[PyNetSparseSSNet] import script" << std::endl;      
    PyObject *pName   = PyUnicode_FromString("Infill_ForwardPass");
    pModule = PyImport_Import(pName);
    if ( !pModule ) {
      throw std::runtime_error("failed to import import module 'Infill_ForwardPass'");
    }
    else {
      std::cout << "[PyNetSparseSSNet] loaded Infill_ForwardPass module" << std::endl;
    }

    pFunc   = PyObject_GetAttrString(pModule,"forwardpass");
    if ( !pFunc ) {
      throw std::runtime_error("failed to load function 'forwardpass' from module");
    }
    else {
      std::cout << "[PyNetSparseSSNet] got function: " << pFunc << std::endl;
    }
    Py_DECREF(pName);

    // weight files
    _weight_file_v.clear();
    for ( auto const& weightfile : weight_file_v )
      _weight_file_v.push_back( weightfile );

  }

  PyNetSparseSSNet::~PyNetSparseSSNet() {

    int is_py_intialized = Py_IsInitialized();
    std::cout << "[PyNetSparseSSNet] attempt to run python finalize. is_intialized=" << is_py_intialized << std::endl;
    try {
      if ( is_py_intialized ) {
	//Py_Finalize(); // sloppy i do not do this, but seg faults during destruction of services if I do
	std::cout << "[PyNetSparseSSNet] successfully (did not) ran finalize" << std::endl;
      }
    }
    catch ( std::exception& e) {
      std::cout << "[PyNetSparseSSNet] error in Py_Finalize: " << e.what() << std::endl;
    }
    
  }

  int PyNetSparseSSNet::run_sparse_ssnet( const std::vector< std::vector<larcv::SparseImage> >& cropped_vv, 
					  const int run, const int subrun, const int event,
					  std::vector<std::vector<larcv::SparseImage> >& results_vv,
					  bool debug ) {

    results_vv.clear();
    
    // for each plane:
    //   make a vector of bsons
    //   send to python script
    //   deserialize and put into results_vv
    for ( size_t planeid=0; planeid<cropped_vv.size(); planeid++ ) {

      std::cout << "[PyNetSparseSSNet] image plane: " << planeid << std::endl;
      
      auto const& cropped_v  = cropped_vv.at(planeid);

      // create list to fill
      PyObject* pList = PyList_New( (Py_ssize_t)cropped_v.size() );

      // fill list with bson objects
      for ( size_t idx=0; idx<cropped_v.size(); idx++ ) {
        
        const larcv::SparseImage& spimg = cropped_v.at(idx);
        PyObject* bson = larcv::json::as_bson_pybytes( spimg, run, subrun, event, idx );
	std::cout << "  [" << planeid << "," << idx << "] add image into python list" << std::endl;

        // set item for previously unitialized list
        PyList_SET_ITEM(pList, (Py_ssize_t)idx, bson );
        // if ( status==-1 ) {
        //   throw std::runtime_error("[PyNetSparseSSNet] trouble setting item for input bson list");
        // }
      }
      std::cout << "[PyNetSparseSSNet] filled bson list plane[" << planeid << "] size=" << PyList_Size(pList) << std::endl;

      PyObject *pWeightpath = PyString_FromString( _weight_file_v.at( planeid ).c_str() );
      PyObject* pPlaneID    = PyInt_FromLong( (long)planeid ); 
      
      std::cout << "[PyNetSparseSSNet] call function: " << pFunc << " weight=" << PyString_AsString( pWeightpath ) << std::endl;
      PyObject *pReturn = PyObject_CallFunctionObjArgs(pFunc,pPlaneID,pList,pWeightpath,NULL);
      std::cout << "python returned: " << pReturn << std::endl;
      
      if (!PyList_Check(pReturn)) {
	throw std::runtime_error("Return from pynet_deploy.inference_mrcnn.forwardpass was no a list");
      }
      
      auto nout = PyList_Size(pReturn);
      
      std::cout << "Number of masks returned from python: " << nout << std::endl;
      std::vector<larcv::SparseImage> out_v;
      for (int iout=0; iout<(int)nout; iout++ ) {
	PyObject* sparseimg_bson = PyList_GetItem(pReturn,(Py_ssize_t)iout);
	int ret_run, ret_subrun, ret_event, ret_id;
        
	larcv::SparseImage sparseout
	  = larcv::json::sparseimg_from_bson_pybytes( sparseimg_bson,
                                                      ret_run, ret_subrun, ret_event, ret_id );
	out_v.emplace_back( std::move(sparseout) );
      }
      
      results_vv.emplace_back( std::move(out_v) );
      
      std::cout << "dereference string arguments/paths" << std::endl;
      Py_DECREF(pWeightpath);
      Py_DECREF(pPlaneID);
      Py_DECREF(pReturn);
      Py_DECREF(pList);

    }//end of loop over plane
    
    std::cout << "done calling SparseSSNet python script" << std::endl;
    return 0;
  }
  

}
}
