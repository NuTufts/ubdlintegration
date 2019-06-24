#ifndef __LARFLOW_H__
#define __LARFLOW_H__

#include "Python.h"

#include <vector>

#include "ServerInterface.h"

#include "larcv/core/DataFormat/Image2D.h"

#include "DataFormat/larflow3dhit.h"

namespace dl {

  class LArFlow : public ServerInterface {

  public:

  LArFlow( zmq::socket_t* broker_socket, zmq::pollitem_t* poller,
	   int client_timeout_secs=300, int max_n_broker_reconnect=3, 
	   int n_max_parts=100, int request_max_tries=3 )
    : ServerInterface( broker_socket, poller, client_timeout_secs,
		       max_n_broker_reconnect, n_max_parts, request_max_tries )
      {}
    
    virtual ~LArFlow() {}
    
    void processCroppedLArFlowViaServer( const std::vector<larcv::Image2D>& cropped_adc_v,
					 const int run, const int subrun, const int event, const float threshold,
					 std::vector<larcv::SparseImage>& flowresults_v, 
					 bool debug );
    
    std::vector<larlite::larflow3dhit> 
      croppedFlow2hits( const std::vector<larcv::SparseImage>& flowresults_v, 
			const std::vector<larcv::Image2D>& wholeview_v,
			const std::string ubcroptrueflow_cfg,
			const float threshold,
			bool debug );

					  
    
  protected:



  };

}

#endif
