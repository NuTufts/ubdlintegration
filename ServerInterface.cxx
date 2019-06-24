#include "ServerInterface.h"

#include "larcv/core/DataFormat/ClusterMask.h"
#include "larcv/core/DataFormat/SparseImage.h"

#include "larcv/core/json/json_utils.h"

namespace dl {

  /**
   * Constructor for ServerInterface class.
   *
   * This class manages sending and receiving images to workers via the Server.
   *
   * @param[in] broker_socket pointer to socket connected to broker.
   * @param[in] poller pollitem_t to indicated to us when messages appear on the socket.
   * @param[in] client_timeout_secs seconds for client to wait for worker/server response.
   * @param[in] max_n_broker_reconnect max number of times the client will reconnect and resend
   *                                   when the client times out.
   * @param[in] n_max_parts maximum number of parts we expect the worker message to contain.
   *
   */
  ServerInterface::ServerInterface( zmq::socket_t* broker_socket, zmq::pollitem_t* poller,
				    int client_timeout_secs, int max_n_broker_reconnect, int n_max_parts,
				    int request_max_tries )
    : _broker(broker_socket),
      _poller(poller),
      _client_timeout_secs(client_timeout_secs),
      _max_n_broker_reconnect(max_n_broker_reconnect),
      _n_max_parts(n_max_parts),
      _request_max_tries( request_max_tries )
  {}

  /**
   * send sparse image vector and receive worker output in same format.
   *
   * @param[in] service_name name of worker service
   * @param[in] outgoing_v  vector of sparse image data to be sent to workers
   * @param[in] incoming_vv vector of vector of sparse image data produced by workers. 
   *                        each inner vector contains the replies to a given image in outgoing_v.
   * @param[in] run run number of image data
   * @param[in] subrun subrun number of image data
   * @param[in] event event number of image data
   * @param[in] expected_reply_ratio  number of reply images for every one outgoing image
   * @param[in] debug print debug statements. default is false.
   * @return int indiciate status. [0] OK; [1] ERROR;
   * 
   */  
  int ServerInterface::sendReceiveSparseImageData( const std::string service_name,
						   const std::vector<larcv::SparseImage>& outgoing_v, 
						   std::vector< std::vector<larcv::SparseImage> >& incoming_vv,
						   const int run, const int subrun, const int event,						  
						   int expected_reply_ratio, bool debug ) {
    // -------------------------------------------------------------------
    // first we need to make the zmq messages

    std::vector< zmq::message_t > zmsg_v;
    
    // loop over input sparseimages and convert into bson and then copy into zmq message
    for ( size_t iimg=0; iimg<outgoing_v.size(); iimg++ ) {
      const larcv::SparseImage& spimg = outgoing_v.at(iimg);
      std::vector<std::uint8_t> bson_v = larcv::json::as_bson( spimg, run, subrun, event, (int)iimg );
      zmq::message_t an_image_msg( bson_v.size() );
      memcpy( an_image_msg.data(), bson_v.data(), bson_v.size() );
      zmsg_v.emplace_back( std::move(an_image_msg) );
    }
    
    // -------------------------------------------------------
    // handle reply

    int n_timeout = 0; // number of times we've timedout waiting for reply
    int ntries    = 0; // number of tries to get complete response

    // send the message according to majortomo protocol
    if ( debug ) {
      std::cout << "ServerInterface::sendReceiveSparseImageData:"
		<< " num images to send = " << zmsg_v.size() 
		<< std::endl;
    }

    size_t nimgs = zmsg_v.size();
    
    std::vector<bool> reply_ok;
    if ( expected_reply_ratio>0 )
      reply_ok.resize( nimgs, false );
    
    std::vector<int>   ok_v( nimgs, 0 ); // track num good replies/images returned
    bool is_complete = false;            // true, when all messages responses collected

    incoming_vv.clear();
    incoming_vv.resize( nimgs );

    while ( !is_complete && ntries<_request_max_tries ) {
      if ( debug ) std::cout << "Receive attempt[" << ntries << "]" << std::endl;
      ntries += 1;

      // send the images
      _send( service_name, zmsg_v, reply_ok, debug );

      // poll for a response
      bool isfinal = false; // final msg received
      std::vector< zmq::message_t > zreply_v; // reply container

      while ( !isfinal ) {

	zmq::poll( _poller, 1, _client_timeout_secs*100000 ); // last argument is in microseconds

	if ( n_timeout<_max_n_broker_reconnect
	     &&  _poller->revents & ZMQ_POLLERR ) {
	  n_timeout += 1;
	  std::cout << "Timed out waiting for response " 
		    << "(" << _client_timeout_secs << " secs)" << std::endl;
	  continue;
	}
	else if ( n_timeout==_max_n_broker_reconnect ) {
	  std::cout << "Too many reconnections to the brokers "
		    << "(" << n_timeout << "): stopped."
		    << std::endl;
	  // we've failed
	  return 1; 
	}
	else if ( _poller->revents & ZMQ_POLLIN ) {

	  // receive multi-part: keep receiving until done
	  // expect reply with at least 4 parts: [empty] [WORKER HEADER] [PARTIAL/FINAL] [DATA ...]

	  // reset timeout counter, as we've successfully communicated with broker
	  n_timeout = 0;
	  
	  int more = 1;                         // more parts coming
	  std::vector< zmq::message_t > part_v; // store parts
	  int nparts = 0;

	  while ( more==1 && nparts<_n_max_parts ) {
	    zmq::message_t reply;
	    _broker->recv( &reply );
	    size_t more_size = sizeof(more);
	    _broker->getsockopt(ZMQ_RCVMORE,&more,&more_size);
	    part_v.emplace_back( std::move(reply) );
	    if ( debug ) std::cout << "    received msg part=" << nparts << " more=" << more << std::endl;
	    nparts++;
	  }
	  if ( debug )  {
	    std::cout << "  Received in batch: " << part_v.size()  << ".  "
		      << "  Expect >= 4 parts."
		      << std::endl;
	  }

	  // parse
	  bool ok = true;
	  if ( part_v.at(0).size()!=0 ) {
	    ok = false;
	    std::cerr << "  first msg not empy!" << std::endl;
	  }
	  std::string reply_header = (char*)part_v.at(1).data();
	  //std::cout << "  reply header: " << reply_header << std::endl;
	  if ( ok && reply_header!="MDPC02" ) {
	    ok = false;
	    std::cerr << "  wrong header." << std::endl;
	  }
	  if ( ok ) {
	    //std::cout << "  [partial/final] = " << (char*)part_v.at(2).data() << std::endl;
	    if ( *((char*)part_v.at(2).data())=='\004' ) {
	      isfinal = true;
	      if ( debug ) std::cout << "  final marker received" << std::endl;
	    }
	    for ( size_t i=3; i<part_v.size(); i++ ) {
	      zreply_v.emplace_back( std::move(part_v[i]) );
	    }
	  }
	  if ( !ok ) {
	    std::cerr << "Error in parser" << std::endl;
	    break;
	  }
	
	} // end of if poll-in found
      }//end of isfinal loop: current reply seems over.


      if ( debug ) std::cout << " Number of replies: " << zreply_v.size() << std::endl;

      // we check if we got all of them back
      int ireply = -1;
      for ( auto& data : zreply_v ) {
	ireply++;

	// convert into vector of uint8_t
	if ( debug ) std::cout << "  copy reply[" << ireply << "] into bson_v " << std::endl;

	std::vector< std::uint8_t > bson( data.size(), 0 );
	memcpy( bson.data(), data.data(), data.size() );

	// convert into image
	if ( debug ) std::cout << "  convert bson[" << ireply << "] into larcv::SparseImage" << std::endl;
	nlohmann::json j = larcv::json::json_from_bson( bson );
	int rrun,rsubrun,revent,reid; // returned IDs
	larcv::json::rseid_from_json( j, rrun, rsubrun, revent, reid );
	larcv::SparseImage img = larcv::json::sparseimg_fromjson( j );

	if ( rrun==run && rsubrun==subrun && revent==event && reid>=0 && reid<(int)nimgs ) {
	  if ( debug ) 
	    std::cout << "  reply[" << ireply << "] has expected ID numbers: "
		      << rrun << ", " << rsubrun << ", " << revent << ", " << reid << std::endl;
	  if ( ok_v[reid]<expected_reply_ratio ) {
	    incoming_vv.at( reid ).emplace_back( std::move(img) );
	    ok_v[reid] = incoming_vv[reid].size();
	    if ( ok_v[reid]==expected_reply_ratio ) {
	      reply_ok[reid] = true; // marks we got all the messsages we expected
	    }
	  }

	}
	else {
	  std::cerr << "  Unexpected (run,subrun,event,id)" << std::endl;
	}
      }//end of zreply_v loop


      // check status of replies
      is_complete = true;
      for ( size_t ii=0; ii<nimgs; ii++ ) {
	if ( !!reply_ok[ii] ) {
	  is_complete = false;
	  std::cerr << "  Missing reply imgid=" << ii << std::endl;
	}
      }
      
    }//end of request loop

    if ( is_complete ) {
      if ( debug ) std::cout << "  Replies complete!" << std::endl;
    }
    else {
      std::cerr << "Incomplete set of images processed" << std::endl;
      return 1;
    }
    
    // return ok status
    return 0;
    
    
  }

  /**
   *  send message to server, skipping messages already w/ good reply.
   *
   *  @param[in] service_name string with the name of the worker service we send our images to.
   *  @param[in] zmsg_v vector of messages containing our image data.
   *  @param[in] reply_ok vector same size as zmsg_v indicating that we have received messages
   *                      for this item. thus, we do not send message if true.
   *  @param[in] debug if true, debug statements printed to standard out.
   *  @return number of images sent
   */
  int ServerInterface::_send( std::string service_name,
			      std::vector<zmq::message_t>& zmsg_v, 
			      std::vector<bool>& reply_ok,
			      bool debug ) {

    // message header codes
    const std::uint8_t empty = 0;
    const char request       = '\002';
    zmq::message_t header_msg (6);
    memcpy (header_msg.data (), "MDPC02", 6);

    
    // header
    // --------
    
    if (debug) std::cout << "  debug: send empty" << std::endl;
    _broker->send( (char*)empty,       0, ZMQ_SNDMORE );
    if (debug) std::cout << "  debug: send header" << std::endl;
    _broker->send( header_msg.data(),  6, ZMQ_SNDMORE );
    if (debug) std::cout << "  debug: send request command" << std::endl;
    _broker->send( (char*)&request, 1, ZMQ_SNDMORE );
    if (debug) std::cout << "  debug: send service name" << std::endl;
    _broker->send( service_name.c_str(), service_name.size(), ZMQ_SNDMORE );

    size_t nimgs = zmsg_v.size();
    // last image
    size_t last_index = 0;
    for ( size_t imsg=0; imsg<nimgs; imsg++ ) {
      if ( reply_ok[imsg] ) continue;
      last_index = imsg;
    }

    // data
    // ------

    int nsent = 0;
    for ( size_t imsg=0; imsg<nimgs; imsg++ ) {
      if ( reply_ok[imsg]>0 ) continue;
      zmq::message_t& zmsg = zmsg_v.at(imsg);
      
      if (imsg!=last_index) {
	if ( debug ) std::cout << "  debug: send img[" << imsg << "]" << std::endl;
	_broker->send( zmsg.data(), zmsg.size(), ZMQ_SNDMORE );
	nsent++;
      }
      else {
	if ( debug ) std::cout << "  debug: send final img[" << imsg << "]" << std::endl;
	_broker->send( zmsg.data(), zmsg.size(), 0 );
	nsent++;
      }
    }
    
    return nsent;
  }

}
