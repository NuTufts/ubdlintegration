#ifndef __INFILL_H__
#define __INFILL_H__

#include "Python.h"

#include <vector>

#include "ServerInterface.h"

#include "larcv/core/DataFormat/Image2D.h"
#include "larcv/core/DataFormat/SparseImage.h"
#include "larcv/core/DataFormat/EventImage2D.h"
#include "larcv/core/DataFormat/EventChStatus.h"

namespace dl {

  class Infill : public ServerInterface {

  public:

  Infill( zmq::socket_t* broker_socket, zmq::pollitem_t* poller,
	   int client_timeout_secs=300, int max_n_broker_reconnect=3, 
	   int n_max_parts=100, int request_max_tries=3 )
    : ServerInterface( broker_socket, poller, client_timeout_secs,
		       max_n_broker_reconnect, n_max_parts, request_max_tries )
      {}
    
    virtual ~Infill() {}
    
    void processWholeImageInfillViaServer( const std::vector<larcv::Image2D>& adc_v,
					   larcv::EventChStatus& ev_chstatus,
					   const int run, const int subrun, const int event, 
					   std::vector<std::vector<larcv::SparseImage> >& adc_crops_vv,
					   std::vector<std::vector<larcv::SparseImage> >& results_vv,
					   std::vector<larcv::Image2D>& stitched_output_v,
					   const float threshold,
					   const std::string infill_crop_cfg,
					   bool debug );
    
    void processSparseCroppedInfillViaServer( const std::vector<std::vector<larcv::SparseImage> >& cropped_vv,
					      const int run, const int subrun, const int event, 
					      std::vector<std::vector<larcv::SparseImage> >& results_vv,
					      const float threshold,
					      bool debug );
    
    void stitchSparseCrops( const std::vector< std::vector<larcv::SparseImage> >& netout_vv,
			    const std::vector<larcv::Image2D>& wholeview_adc_v,
			    larcv::EventChStatus& ev_status,
			    std::vector< larcv::Image2D >& mergedout_v,
			    bool debug );
					  
    
  protected:



  };

}

#endif
