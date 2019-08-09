#include "PyNet_SparseInfill.h"

#include "larcv/core/DataFormat/SparseImage.h"
#include "larcv/core/json/json_utils.h"
#include "larcv/core/PyUtil/PyUtils.h"

namespace ubcv {
namespace ubdlintegration {

  PyNetSparseInfill::PyNetSparseInfill( const std::vector< std::string >& weight_file_v )
    : pModule(nullptr),
      pFunc(nullptr)
  {

    std::cout << "[PyNetSparseInfill] py initialize" << std::endl;
    Py_Initialize();

    std::cout << "[PyNetSparseInfill] import numpy array" << std::endl;
    larcv::SetPyUtil();

    std::cout << "[PyNetSparseInfill] import script" << std::endl;      
    PyObject *pName   = PyUnicode_FromString("Infill_ForwardPass");
    pModule = PyImport_Import(pName);
    if ( !pModule ) {
      throw std::runtime_error("failed to import import module 'Infill_ForwardPass'");
    }
    else {
      std::cout << "[PyNetSparseInfill] loaded Infill_ForwardPass module" << std::endl;
    }

    pFunc   = PyObject_GetAttrString(pModule,"forwardpass");
    if ( !pFunc ) {
      throw std::runtime_error("failed to load function 'forwardpass' from module");
    }
    else {
      std::cout << "[PyNetSparseInfill] got function: " << pFunc << std::endl;
    }
    Py_DECREF(pName);

    // weight files
    _weight_file_v.clear();
    for ( auto const& weightfile : weight_file_v )
      _weight_file_v.push_back( weightfile );

  }

  PyNetSparseInfill::~PyNetSparseInfill() {

    int is_py_intialized = Py_IsInitialized();
    std::cout << "[PyNetSparseInfill] attempt to run python finalize. is_intialized=" << is_py_intialized << std::endl;
    try {
      if ( is_py_intialized ) {
	Py_Finalize();
	std::cout << "[PyNetSparseInfill] successfully ran finalize" << std::endl;
      }
    }
    catch ( std::exception& e) {
      std::cout << "[PyNetSparseInfill] error in Py_Finalize: " << e.what() << std::endl;
    }
    
  }

  int PyNetSparseInfill::run_sparse_cropped_infill( const std::vector< std::vector<larcv::SparseImage> >& cropped_vv, 
                                                    const int run, const int subrun, const int event,
                                                    std::vector<std::vector<larcv::SparseImage> >& results_vv,
                                                    bool debug ) {

    results_vv.clear();
    
    // for each plane:
    //   make a vector of bsons
    //   send to python script
    //   deserialize and put into results_vv
    for ( size_t planeid=0; planeid<cropped_vv.size(); planeid++ ) {

      std::cout << "[PyNetSparseInfill] image plane: " << planeid << std::endl;
      
      auto const& cropped_v  = cropped_vv.at(planeid);

      // create list to fill
      PyObject* pList = PyList_New( (Py_ssize_t)cropped_v.size() );

      // fill list with bson objects
      for ( size_t idx=0; idx<cropped_v.size(); idx++ ) {
        
        const larcv::SparseImage& spimg = cropped_v.at(idx);
        PyObject* bson = larcv::json::as_bson_pybytes( spimg, run, subrun, event, idx );
	std::cout << "  [" << idx << "] add cropped image into python list" << std::endl;

        // set item for previously unitialized list
        PyList_SET_ITEM(pList, (Py_ssize_t)idx, bson );
        // if ( status==-1 ) {
        //   throw std::runtime_error("[PyNetSparseInfill] trouble setting item for input bson list");
        // }
      }
      std::cout << "[PyNetSparseInfill] filled bson list plane[" << planeid << "] size=" << PyList_Size(pList) << std::endl;

      PyObject *pWeightpath   = PyString_FromString( _weight_file_v.at( planeid ).c_str() );
      
      std::cout << "[PyNetSparseInfill] call function: " << pFunc << " weight=" << PyString_AsString( pWeightpath ) << std::endl;
      PyObject *pReturn = PyObject_CallFunctionObjArgs(pFunc,pList,pWeightpath,NULL);
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
      Py_DECREF(pReturn);
      Py_DECREF(pList);

    }

    std::cout << "done calling SparseInfill python script" << std::endl;
    return 0;
  }
  

}
}
