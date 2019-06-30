#ifndef __COSMIC_MRCNN_H__
#define __COSMIC_MRCNN_H__

#include "Python.h"

#include <vector>

#include "ServerInterface.h"

#include "larcv/core/DataFormat/Image2D.h"
#include "larcv/core/DataFormat/ClusterMask.h"

namespace dl {

  class CosmicMRCNN : public ServerInterface {

  public:

  CosmicMRCNN( zmq::socket_t* broker_socket, zmq::pollitem_t* poller,
	       int client_timeout_secs=300, int max_n_broker_reconnect=3, 
	       int n_max_parts=100, int request_max_tries=3 )
    : ServerInterface( broker_socket, poller, client_timeout_secs,
		       max_n_broker_reconnect, n_max_parts, request_max_tries )
      {}
    
    virtual ~CosmicMRCNN() {}
    
    void processViaServer( const std::vector<larcv::Image2D>& wholeview_adc_v,
			   const int run, const int subrun, const int event, 
			   std::vector< std::vector<larcv::ClusterMask> >& result_mask_vv, 
			   bool debug );
        
  protected:



  };

}

#endif
