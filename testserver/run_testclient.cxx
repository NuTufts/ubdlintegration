//
//  Hello World client in C++
//  Connects REQ socket to tcp://localhost:5555
//  Sends "Hello" to server, expects "World" back
//
#include <zmq.hpp>
#include <string>
#include <iostream>

int main ()
{
  //  Prepare our context and socket
  zmq::context_t context (1);
  zmq::socket_t socket (context, ZMQ_REQ);

  std::cout << "Connecting to hello world server…" << std::endl;
  //socket.connect ("tcp://localhost:5555");
  socket.connect ("tcp://nudot.lns.mit.edu:6000");

  const std::uint8_t  empty   = 0;
  const std::uint8_t request  = 2;
  char client_header[7];
  sprintf(client_header,"MDPC02");
  zmq::message_t header_msg (6);
  memcpy (header_msg.data (), "MDPC02", 6);
  char request = '\002';


  //  Do 10 requests, waiting each time for a response
  for (int request_nbr = 0; request_nbr != 10; request_nbr++) {

    // service name
    char service_name[20];
    sprintf(service_name,"ubssnet_plane%d",request_nbr);

    // send the message parts    
    std::cout << "  debug: send empty" << std::endl;
    //socket.send( (char*)&empty,     0, ZMQ_SNDMORE );
    std::cout << "  debug: send header" << std::endl;
    socket.send( header_msg.data(), 6, ZMQ_SNDMORE );
    std::cout << "  debug: send request command" << std::endl;
    socket.send( (char*)&request2, 1, ZMQ_SNDMORE );
    std::cout << "  debug: send service name" << std::endl;
    socket.send( service_name,  14, ZMQ_SNDMORE );

    zmq::message_t request (5);
    memcpy (request.data (), "Hello", 5);
    std::cout << "Sending Hello " << request_nbr << "…" << std::endl;
    socket.send (request.data(),5,0);

    //  Get the reply.
    zmq::message_t reply;
    socket.recv (&reply);
    std::cout << "Received World " << request_nbr << std::endl;
  }
  return 0;
}
