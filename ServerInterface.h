#ifndef __SERVER_INTERFACE_H__
#define __SERVER_INTERFACE_H__


/**
 *
 * Class to handle server communication for different networks.
 *
 * Handle communication with broker along with packing/unpacking message.
 * Also, deal with replies from broker/worker.
 * Of course, that means that somewhat restricted input/output data.
 * Input data must be vector of {images,sparseimages}.
 * Will expect worker to return a vector of {images,sparseimages}.
 * Note, either/or, no plans to handle hybrid data at the moment.
 *
 */

#include "Python.h"

#include <vector>
#include <string>

// zmq-c++
#include "zmq.hpp"

// larcv
#include "larcv/core/DataFormat/Image2D.h"
#include "larcv/core/DataFormat/SparseImage.h"
#include "larcv/core/DataFormat/ClusterMask.h"


namespace dl {

  class ServerInterface {
    
  public:

    ServerInterface( zmq::socket_t* broker_socket, zmq::pollitem_t* poller,
		     int client_timeout_secs=300, int max_n_broker_reconnect=3, 
		     int n_max_parts=100, int request_max_tries=3 );

    virtual ~ServerInterface() {}
    
    // template send/receive function
    template < class InType, class OutType >
      int sendReceiveData( const std::string service_name,
			   const std::vector< InType >& outgoing_v, 
			   std::vector< std::vector< OutType > >& incoming_vv,
			   const int run, const int subrun, const int event,						  
			   int expected_reply_ratio, bool debug );

    
  protected:

    zmq::socket_t*   _broker; // socket to server, class does not own this pointer.
    zmq::pollitem_t* _poller; // poller for the server socket. class does not own this.

    int _client_timeout_secs;
    int _max_n_broker_reconnect;
    int _n_max_parts;
    int _request_max_tries;

    int _send( std::string service_name,
	       std::vector<zmq::message_t>& zmsg_v, 
	       std::vector<bool>& reply_ok,
	       bool debug );


  };

  
}

#endif
