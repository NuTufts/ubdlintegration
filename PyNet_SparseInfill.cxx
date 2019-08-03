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

    std::cout << "py initialize" << std::endl;
    Py_Initialize();

    std::cout << "import numpy array" << std::endl;
    larcv::SetPyUtil();

    std::cout << "import script" << std::endl;      
    PyObject *pName   = PyUnicode_FromString("");
    pModule = PyImport_Import(pName);
    std::cout << "import module: " << pModule << std::endl;
    pFunc   = PyObject_GetAttrString(pModule,"forwardpass");
    std::cout << "got function: " << pFunc << std::endl;
    Py_DECREF(pName);

    // weight files
    _weight_file_v.clear();
    for ( auto const& weightfile : weight_file_v )
      _weight_file_v.push_back( weightfile );

  }

  PyNetSparseInfill::~PyNetSparseInfill() {

    std::cout << "python finalize" << std::endl;
    Py_Finalize();
    
  }

  int PyNetSparseInfill::run_sparse_cropped_infill( const std::vector< std::vector<larcv::SparseImage> >& cropped_vv, 
                                                    const int run, const int subrun, const int event,
                                                    std::vector<std::vector<larcv::SparseImage> >& results_vv,
                                                    const float threshold,
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

        // set item for previously unitialized list
        int status = PyList_SET_ITEM(pList, (Py_ssize_t)idx, bson );
        if ( status==-1 ) {
          throw std::runtime_error("[PyNetSparseInfill] trouble setting item for input bson list");
        }
      }
      
      PyObject *pWeightpath   = PyUnicode_FromString( _weight_file_v.at( planeid ).c_str() );
      
      std::cout << "call function: " << pFunc << " bson=" << bson <<" weight=" << pWeightpath << std::endl;
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
        
	larcv::ClusterMask sparseout
	  = larcv::json::sparseimg_from_bson_pybytes( sparseimg_bson,
                                                      ret_run, ret_subrun, ret_event, ret_id );
	masks_v.emplace_back( std::move(sparseout) );
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
