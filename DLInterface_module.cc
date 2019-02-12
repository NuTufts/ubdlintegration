////////////////////////////////////////////////////////////////////////
// Class:       DLInterface
// Plugin Type: producer (art v2_05_01)
// File:        DLInterface_module.cc
//
// Generated at Wed Aug 29 05:49:09 2018 by Taritree,,, using cetskelgen
// from cetlib version v1_21_00.
////////////////////////////////////////////////////////////////////////

#include "art/Framework/Core/EDProducer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Principal/Run.h"
#include "art/Framework/Principal/SubRun.h"
#include "canvas/Utilities/InputTag.h"
#include "fhiclcpp/ParameterSet.h"
#include "messagefacility/MessageLogger/MessageLogger.h"

#include <memory>

// ubcv
#include "ubcv/LArCVImageMaker/LArCVSuperaDriver.h"
#include "ubcv/LArCVImageMaker/ImageMetaMaker.h"
#include "ubcv/LArCVImageMaker/SuperaMetaMaker.h"
#include "ubcv/LArCVImageMaker/SuperaWire.h"
#include "ubcv/LArCVImageMaker/LAr2Image.h"

// larcv
#include "larcv/core/Base/larcv_base.h"
#include "larcv/core/Base/PSet.h"
#include "larcv/core/Base/LArCVBaseUtilFunc.h"
#include "larcv/core/DataFormat/EventImage2D.h"
#include "larcv/core/DataFormat/Image2D.h"
#include "larcv/core/DataFormat/ImageMeta.h"
#include "larcv/core/DataFormat/ROI.h"
#include "larcv/core/json/json_utils.h"
#include "ublarcvapp/UBImageMod/UBSplitDetector.h"

// ROOT
#include "TMessage.h"

// zmq-c++
#include "zmq.hpp"

#ifdef HAS_TORCH
// TORCH
#include <torch/script.h>
#include <torch/torch.h>

// larcv torch utils
#include "larcv/core/TorchUtil/TorchUtils.h"
#endif

class DLInterface;


class DLInterface : public art::EDProducer, larcv::larcv_base {
public:
  explicit DLInterface(fhicl::ParameterSet const & p);
  // The compiler-generated destructor is fine for non-base
  // classes without bare pointers or other resource use.

  typedef enum { kServer=0, kPyTorchCPU, kTensorFlowCPU, kDummyServer } NetInterface_t;

  // Plugins should not be copied or assigned.
  DLInterface(DLInterface const &) = delete;
  DLInterface(DLInterface &&) = delete;
  DLInterface & operator = (DLInterface const &) = delete;
  DLInterface & operator = (DLInterface &&) = delete;

  // Required functions.
  void produce(art::Event & e) override;
  void beginJob() override;
  void endJob() override;
  
private:

  // Declare member data here.

  // supera/image formation
  larcv::LArCVSuperaDriver _supera;  
  std::string _wire_producer_name;
  std::string _supera_config;
  int runSupera( art::Event& e, std::vector<larcv::Image2D>& wholeview_imgs );

  // image processing/splitting/cropping
  larcv::UBSplitDetector _imagesplitter;
  bool        _save_detsplit_input;
  bool        _save_detsplit_output;
  std::vector<larcv::Image2D> _splitimg_v;              //< container holding split images
  int runWholeViewSplitter( const std::vector<larcv::Image2D>& wholeview_v, 
			    std::vector<larcv::Image2D>& splitimg_v,
			    std::vector<larcv::ROI>&     splitroi_v );
  
  // the network interface to run
  NetInterface_t _interface;

  // interface: pytorch cpu
  std::string _pytorch_net_script;
#ifdef HAS_TORCH
  std::shared_ptr<torch::jit::script::Module> _module;  //< pointer to pytorch network
#endif
  void loadNetwork_PyTorchCPU();
  int runPyTorchCPU( const std::vector<larcv::Image2D>& wholeview_v,
		     std::vector<larcv::Image2D>& splitimg_v, 
		     std::vector<larcv::ROI>& splitroi_v, 
		     std::vector<larcv::Image2D>& netout_v );
  

  // interface: server (common)
  zmq::context_t* _context;
  zmq::socket_t*  _socket; 
  void openServerSocket();
  void closeServerSocket();
  size_t serializeImages( const size_t nplanes,
			  const std::vector<larcv::Image2D>& img_v,   
			  std::vector< std::vector<unsigned char> >& msg_img_v,
			  std::vector<std::string>& msg_meta_v,
			  std::vector<std::string>& msg_name_v );


  // interface: ssnet gpu server
  int runSSNetServer( const std::vector<larcv::Image2D>& wholeview_v, 
		      std::vector<larcv::Image2D>& splitimg_v, 
		      std::vector<larcv::ROI>& splitroi_v, 
		      std::vector<larcv::Image2D>& netout_v );

  // interface: dummy server (for debug)
  int runDummyServer( art::Event& e );
  void sendDummyMessage();

  // merger/post-processing
  void mergeSSNetOutput( const std::vector<larcv::Image2D>& wholeview_v, 
			 const std::vector<larcv::ROI>& splitroi_v, 
			 const std::vector<larcv::Image2D>& netout_v,
			 std::vector<larcv::Image2D>& merged );
  

};

/**
 * constructor: configuration module
 *
 */
DLInterface::DLInterface(fhicl::ParameterSet const & p)
  : larcv::larcv_base("DLInterface_module")
    // Initialize member data here.
{
  // Call appropriate produces<>() functions here.
  _wire_producer_name = p.get<std::string>("WireProducerName");
  _supera_config      = p.get<std::string>("SuperaConfigFile");
  std::string inter   = p.get<std::string>("NetInterface");
  if ( inter=="Server" )
    _interface = kServer;
  else if ( inter=="Pytorch" )
    _interface = kPyTorchCPU;
  else if ( inter=="TensorFlow" )
    _interface = kTensorFlowCPU;
  else if ( inter=="DummyServer" ) // for debugging
    _interface = kDummyServer;
  else {
    throw cet::exception("DLInterface") << "unrecognized network interface, " << inter << ". "
					<< "choices: { Server, Pytorch, TensorFlow }"
					<< std::endl;
  }

  if ( _interface==kPyTorchCPU )
    throw cet::exception("DLInterface") << "PytorchCPU interface not yet implemented" << std::endl;
  if ( _interface==kTensorFlowCPU )
    throw cet::exception("DLInterface") << "TensorFlowCPU interface not yet implemented" << std::endl;

  _save_detsplit_input  = p.get<bool>("SaveDetsplitImagesInput");
  _save_detsplit_output = p.get<bool>("SaveNetOutSplitImagesOutput");


  std::string supera_cfg;
  cet::search_path finder("FHICL_FILE_PATH");
  if( !finder.find_file(_supera_config, supera_cfg) )
    throw cet::exception("DLInterface") << "Unable to find supera cfg in "  << finder.to_string() << "\n";

  // check cfg content top level
  larcv::PSet main_cfg = larcv::CreatePSetFromFile(_supera_config).get<larcv::PSet>("ProcessDriver");

  // get list of processors
  std::vector<std::string> process_names = main_cfg.get< std::vector<std::string> >("ProcessName");
  std::vector<std::string> process_types = main_cfg.get< std::vector<std::string> >("ProcessType");
  larcv::PSet              process_list  = main_cfg.get<larcv::PSet>("ProcessList");
  larcv::PSet              split_cfg     = main_cfg.get<larcv::PSet>("UBSplitConfig"); 
  
  if ( process_names.size()!=process_types.size() ) {
    throw std::runtime_error( "Number of Supera ProcessName(s) and ProcessTypes(s) is not the same." );
  }

  // configure supera (convert art::Event::CalMod -> Image2D)
  _supera.configure(_supera_config);

  // configure image splitter (fullimage into detsplit image)
  _imagesplitter.configure( split_cfg );

  // get the path to the saved ssnet
  _pytorch_net_script = p.get<std::string>("PyTorchNetScript");


  // verbosity
  int verbosity = p.get<int>("Verbosity",2);
  set_verbosity( (::larcv::msg::Level_t) verbosity );
  _supera.set_verbosity( (::larcv::msg::Level_t) verbosity );

}

/** 
 * produce DL network products
 *
 */
void DLInterface::produce(art::Event & e)
{

  // get the wholeview images for the network
  std::vector<larcv::Image2D> wholeview_v;
  int nwholeview_imgs = runSupera( e, wholeview_v );
  std::cout << "number of wholeview images: " << nwholeview_imgs << std::endl;

  // we often have to pre-process the image, e.g. split it.
  // eventually have options here. But for now, wholeview splitter
  std::vector<larcv::Image2D> splitimg_v;
  std::vector<larcv::ROI>     splitroi_v;
  int nsplit_imgs = runWholeViewSplitter( wholeview_v, splitimg_v, splitroi_v );
  std::cout << "number of split images: " << nsplit_imgs << std::endl;

  // containers for outputs
  std::vector<larcv::Image2D> netout_v;

  // run network (or not)

  int status = 0;
  switch (_interface) {
  case kDummyServer:
  case kTensorFlowCPU:
    status = runDummyServer(e);
    break;
  case kPyTorchCPU:
    status = runPyTorchCPU( wholeview_v, splitimg_v, splitroi_v, netout_v );
    break;
  case kServer:
    status = runSSNetServer( wholeview_v, splitimg_v, splitroi_v, netout_v );
    break;
  }

  if ( status!=0 )
    throw cet::exception("DLInterface") << "Error running network" << std::endl;

  // merge the output
  std::vector<larcv::Image2D> merged;
  mergeSSNetOutput( wholeview_v, splitroi_v, netout_v, merged );

  // prepare the output
  larcv::IOManager& io = _supera.driver().io_mutable();
  
  // save the wholeview images back to the supera IO
  larcv::EventImage2D* ev_imgs  = (larcv::EventImage2D*) io.get_data( larcv::kProductImage2D, "wire" );
  std::cout << "wire eventimage2d=" << ev_imgs << std::endl;
  ev_imgs->Emplace( std::move(wholeview_v) );

  // save detsplit input
  larcv::EventImage2D* ev_splitdet = nullptr;
  if ( _save_detsplit_input ) {
    ev_splitdet = (larcv::EventImage2D*) io.get_data( larcv::kProductImage2D, "detsplit" );
    ev_splitdet->Emplace( std::move(splitimg_v) );
  }

  // save detsplit output
  larcv::EventImage2D* ev_netout_split = nullptr;
  if ( _save_detsplit_output ) {
    ev_netout_split = (larcv::EventImage2D*) io.get_data( larcv::kProductImage2D, "netoutsplit" );
    ev_netout_split->Emplace( std::move(netout_v) );
  }

  // save merged out
  
  // save entry
  std::cout << "saving entry" << std::endl;
  io.save_entry();
  
  // we clear entries ourselves
  std::cout << "clearing entry" << std::endl;
  io.clear_entry();

}

int DLInterface::runDummyServer( art::Event& e ) {
  sendDummyMessage();
  return 0;
}

/**
 * have supera make larcv images
 *
 * get recob::Wire from art::Event, pass it to supera
 * the _supera object will contain the larcv IOManager which will have images stored
 *
 * @param[in] e art::Event with wire data
 * @return int number of images to process by network
 *
 */
int DLInterface::runSupera( art::Event& e, std::vector<larcv::Image2D>& wholeview_imgs ) {

  //
  // set data product
  // 
  // :wire  
  art::Handle<std::vector<recob::Wire> > data_h;
  //  handle sub-name
  if(_wire_producer_name.find(" ")<_wire_producer_name.size()) {
    e.getByLabel(_wire_producer_name.substr(0,_wire_producer_name.find(" ")),
		 _wire_producer_name.substr(_wire_producer_name.find(" ")+1,_wire_producer_name.size()-_wire_producer_name.find(" ")-1),
		 data_h);
  }else{ e.getByLabel(_wire_producer_name, data_h); }
  if(!data_h.isValid()) { 
    std::cerr<< "Attempted to load recob::Wire data: " << _wire_producer_name << std::endl;
    throw ::larcv::larbys("Could not locate data!"); 
  }
  _supera.SetDataPointer(*data_h,_wire_producer_name);

  // execute supera
  bool autosave_entry = false;
  std::cout << "process event: (" << e.id().run() << "," << e.id().subRun() << "," << e.id().event() << ")" << std::endl;
  _supera.process(e.id().run(),e.id().subRun(),e.id().event(), autosave_entry);

  // get the images
  auto ev_imgs  = (larcv::EventImage2D*) _supera.driver().io_mutable().get_data( larcv::kProductImage2D, "wire" );
  ev_imgs->Move( wholeview_imgs );
  // wholeview_imgs.clear();
  // for ( auto const& img : ev_imgs->Image2DArray() ) {
  //   wholeview_imgs.push_back( img );
  // }
  
  return wholeview_imgs.size();
}

/**
 * take wholeview images (one per plane in vector) and turn into subimage crops (many per plane in vector)
 *
 * passes the images into the UBSplitDetector object (_imagesplitter)
 *
 * @param[in] wholeview_v vector of whole-view images. one per plane.
 * @param[inout] splitimg_v vector of cropped images. many per plane.
 * @param[inout] splitroi_v vector ROIs for split images. many  per plane.
 *
 **/
int DLInterface::runWholeViewSplitter( const std::vector<larcv::Image2D>& wholeview_v, 
				       std::vector<larcv::Image2D>& splitimg_v,
				       std::vector<larcv::ROI>&     splitroi_v ) {
  
  // //
  // // define the image meta and image
  // //
  // std::vector<larcv::ImageMeta> meta_v;
  // for ( auto const& img : wholeview_v ) {
  //   meta_v.push_back( img.meta() );
  // }
  
  //
  // debug: dump the meta definitions for the wholeview images
  std::cout << "DLInterface: superawire images made " << wholeview_v.size() << std::endl;
  for (auto& img : wholeview_v ) {
    std::cout << img.meta().dump() << std::endl;
  }  
  
  //
  // split image into subregions
  //
  splitimg_v.clear();
  splitroi_v.clear();
  try {
    _imagesplitter.process( wholeview_v, splitimg_v, splitroi_v );
  }
  catch (std::exception& e ) {
    throw cet::exception("DLInterface") << "error splitting image: " << e.what() << std::endl;
  }

  return splitimg_v.size();

}

/**
 * 
 * serialize image into binary-json message + strings describing image meta
 *
 * @param[in] nplanes number of input plane images
 * @param[in] img_v individual (cropped/split) images to serialize
 * @param[inout] msg_img_v binary json message for each image
 * @param[inout] msg_meta_v string describing meta
 * @param[inout] msg_name_v string describing name
 * 
 */
size_t DLInterface::serializeImages( const size_t nplanes,
				     const std::vector<larcv::Image2D>& img_v,   
				     std::vector< std::vector<unsigned char> >& msg_img_v,
				     std::vector<std::string>& msg_meta_v,
				     std::vector<std::string>& msg_name_v ) {
  
  size_t nimgs = img_v.size();
  size_t nsets = nimgs/nplanes;
  msg_img_v.resize(  nimgs );
  msg_meta_v.resize( nimgs );
  msg_name_v.resize( nimgs );

  size_t img_msg_totsize = 0.;
  
  for (size_t iset=0; iset<nsets; iset++ ) {
    for (size_t p=0; p<nplanes; p++) {
      size_t iimg = nplanes*iset + p;
      const larcv::Image2D& img = img_v[ iimg ];

      char zname[100];
      sprintf( zname, "croicropped_p%d_iset%d_iimg%d", (int)p, (int)iset, (int)iimg );
      
      std::string msg_name = zname;
      std::string msg_meta = img.meta().dump();

      msg_img_v[ iimg ]  = larcv::json::as_bson( img );
      msg_meta_v[ iimg ] = img.meta().dump();
      msg_name_v[ iimg ] = zname;
      
      img_msg_totsize += msg_img_v[iimg].size();
    }
  }
  
  std::cout << "Created " << msg_img_v.size() << " messages. Total image message size: " << img_msg_totsize << " (chars)" << std::endl;
  return img_msg_totsize;
}

/**
 * 
 * run ssnet via GPU server
 *
 * 
 */
int DLInterface::runSSNetServer( const std::vector<larcv::Image2D>& wholeview_v, 
				 std::vector<larcv::Image2D>& splitimg_v, 
				 std::vector<larcv::ROI>& splitroi_v, 
				 std::vector<larcv::Image2D>& netout_v ) {
  int status = 0;
  return status;
}


/**
 * 
 * run networking using pytorch CPU C++ interface
 *
 * 
 */
int DLInterface::runPyTorchCPU( const std::vector<larcv::Image2D>& wholeview_v, 
				std::vector<larcv::Image2D>& splitimg_v, 
				std::vector<larcv::ROI>& splitroi_v, 
				std::vector<larcv::Image2D>& netout_v ) {

#ifdef HAS_TORCH

  int status = 0;
  size_t nplanes = whileview_v.size();
  size_t nimgs   = splitimg_v.size();
  size_t nsets   = nimgs/nplanes;

  std::vector<torch::jit::IValue> inputs;
  inputs.reserve(nimgs+1);
  for (int iimg=0; iimg<nimgs; iimg++ ) {
    //larcv::ROI& roi = splitroi_v[iimg];
    larcv::Image2D& img = splitimg_v[ iimg ];
    inputs.push_back( larcv::torchutils::as_tensor( img ).reshape( {1,1,(int)img.meta().cols(),(int)img.meta().rows()} ) );
  }
  // debug
  std::cout << "Converted the data: nimgs[plane2]=" << inputs.size() << std::endl;
  std::cout << "  shape=" 
  	    << inputs.front().size(0) << ","
  	    << inputs.front().size(1) << ","
  	    << inputs.front().size(2) << ","
  	    << inputs.front().size(3)
  	    << std::endl;


  //run the net!
  int iimg = 0;
  netout_v.clear();
  netout_v.reserve( nimgs+1 );
  for ( auto& data : inputs ) {
    at::Tensor output;
    try {
      output = _module->forward(data).toTensor();
      //std::cout << "network produced: " << output.size(0) << "," << output.size(1) << "," << output.size(2) << std::endl;
    }
    catch (std::exception& e) {
      throw cet::exception("DLInterface") << "module error while running data: " << e.what() << std::endl;
    }
    // as img2d
    larcv::Image2D imgout( splitimg_v[iimg].meta() );
    netout_v.emplace_back( std::move(imgout) );
  }

  
  // talk to the socket
  // for ( size_t imsg=0; imsg<msg_img_v.size(); imsg++ ) {

  //   // first the name
  //   std::cout << "sending name[" << imsg << "] ... ";
  //   _socket->send( msg_name_v[imsg].c_str(), msg_name_v[imsg].length(), ZMQ_SNDMORE );
  //   std::cout << " done" << std::endl;

  //   // meta
  //   std::cout << "sending meta[" << imsg << "] ... ";
  //   _socket->send( msg_meta_v[imsg].c_str(), msg_meta_v[imsg].length(), ZMQ_SNDMORE );
  //   std::cout << " done" << std::endl;

  //   // image
  //   std::cout << "sending image[" << imsg << "] ... size=" << msg_img_v[imsg].size() << " ... ";
  //   if (imsg+1!=msg_img_v.size()) {
  //     _socket->send( msg_img_v[imsg].data(), msg_img_v[imsg].size(), ZMQ_SNDMORE );
  //     std::cout << " done" << std::endl;
  //   }
  //   else {
  //     _socket->send( msg_img_v[imsg].data(), msg_img_v[imsg].size() );
  //     std::cout << "last message sent" << std::endl;
  //   }
    
  // }
  // std::cout << "Event messages Sent!" << std::endl;
  
  // zmq::message_t reply;
  // _socket->recv (&reply);
  // std::cout << "Received: " << reply.data() << std::endl;

  // std::cout << "saving entry" << std::endl;
  // _supera.driver().io_mutable().save_entry();
  
  // std::cout << "clearing entry" << std::endl;
  // _supera.driver().io_mutable().clear_entry();
  // std::cout << "Remaining: " <<   ((larcv::EventImage2D*) _supera.driver().io_mutable().get_data( larcv::kProductImage2D, "wire" ))->Image2DArray().size() << std::endl;
#endif // HAS_TORCH

  return 0;
}


/**
 * 
 * merge ssnet output into a wholeview image
 *
 * 
 */
void DLInterface::mergeSSNetOutput( const std::vector<larcv::Image2D>& wholeview_v, 
				    const std::vector<larcv::ROI>& splitroi_v, 
				    const std::vector<larcv::Image2D>& netout_v,
				    std::vector<larcv::Image2D>& merged ) {
  merged.clear();
  for ( auto const& img : wholeview_v ) {
    larcv::Image2D mergeout( img.meta() );
    mergeout.paint(0);
    merged.emplace_back( std::move(mergeout) );
  }

}

/**
 * 
 * overridden method: setup class before job is run
 *
 * 
 */
void DLInterface::beginJob()
{
  _supera.initialize();

  switch( _interface ) {
  case kServer:
  case kDummyServer:
    openServerSocket();
    break;
  case kPyTorchCPU:
#ifdef HAS_TORCH
    try {
      _module = torch::jit::load( _pytorch_net_script );
    }
    catch (...) {
      throw cet::exception("DLInterface") << "Could not load model from " << _pytorch_net_script << std::endl;
    }
    if ( _module==nullptr )
      throw cet::exception("DLInterface") << "model loaded as NULL " << _pytorch_net_script << std::endl;
    std::cout << "Network Loaded" << std::endl;
#endif
    break;
  case kTensorFlowCPU:
    break;
  default:
    _context = nullptr;
    _socket  = nullptr;
    break;
  }

}

/**
 * 
 * overridden method: setup class after all events are run
 *
 * 
 */
void DLInterface::endJob()
{
  std::cout << "DLInterface::endJob -- start" << std::endl;
  std::cout << "DLinterface::endJob -- finalize IOmanager" << std::endl;
  _supera.finalize();
  std::cout << "DLInterface::endJob -- finished" << std::endl;
}

/**
 * 
 * load pytorch network from torch script file
 *
 * 
 */
void DLInterface::loadNetwork_PyTorchCPU() {
#ifdef HAS_TORCH
  std::cout << "Loading network from " << _pytorch_net_script << " .... " << std::endl;
  
  std::shared_ptr<torch::jit::script::Module> module = nullptr;
  try {
    _module = torch::jit::load( _pytorch_net_script );
  }
  catch (...) {
    throw cet::exception("DLInterface") << "Could not load model from " << _pytorch_net_script << std::endl;
  }
  if ( _module==nullptr )
    throw cet::exception("DLInterface") << "model loaded as NULL " << _pytorch_net_script << std::endl;
  std::cout << "Network Loaded" << std::endl;
#endif
}


/**
 * 
 * open the server socket
 *
 * 
 */
void DLInterface::openServerSocket() {

  std::cout << "DLInterface: open server socket" << std::endl;
  
  // open the zmq socket
  _context = new zmq::context_t(1);
  _socket  = new zmq::socket_t( *_context, ZMQ_REQ );
  char identity[10];
  sprintf(identity,"larbys00"); //to-do: replace with pid or some unique identifier
  _socket->setsockopt( ZMQ_IDENTITY, identity, 8 );

  if ( _interface==kDummyServer ) 
    _socket->connect("tcp://localhost:5555");
  else
    throw cet::exception("DLInterface") << "connection for gpu-server not defined yet" << std::endl;

}

/**
 * 
 * close the server socket
 *
 * 
 */
void DLInterface::closeServerSocket() {

  std::cout << "DLInterface: close server socket" << std::endl;

  // close zmq socket
  if ( _socket )
    delete _socket;
  if ( _context )
    delete _context;

  _socket  = nullptr;
  _context = nullptr;
}

/**
 * 
 * for debug: send a dummy message to the dummy server
 *
 * 
 */
void DLInterface::sendDummyMessage() {

  std::cout << "DLInterface: close server socket" << std::endl;

  //  Do 10 requests, waiting each time for a response
  for (int request_nbr = 0; request_nbr != 10; request_nbr++) {
    zmq::message_t request (5);
    memcpy (request.data (), "Hello", 5);
    std::cout << "Sending Hello " << request_nbr << "…" << std::endl;
    _socket->send (request);
    
    //  Get the reply.
    zmq::message_t reply;
    _socket->recv (&reply);
    std::cout << "Received World " << request_nbr << std::endl;
  }
  
  std::cout << "end of sendDummyMessage()" << std::endl;
}


DEFINE_ART_MODULE(DLInterface)
