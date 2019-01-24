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
#include "ubcv/LArCVImageMaker/ImageMetaMaker.h"
#include "ubcv/LArCVImageMaker/SuperaMetaMaker.h"
#include "ubcv/LArCVImageMaker/SuperaWire.h"
#include "ubcv/LArCVImageMaker/LAr2Image.h"

// larcv
#include "Base/PSet.h"
#include "Base/LArCVBaseUtilFunc.h"
#include "DataFormat/Image2D.h"
#include "DataFormat/ImageMeta.h"
#include "DataFormat/ROI.h"
#include "ImageMod/UBSplitDetector.h"
#include "TorchUtil/TorchUtils.h"
//#include "serializer/serializer.h"

// ROOT
#include "TMessage.h"

// zmq-c++
//#include "zmq.hpp"

// TORCH
#include <torch/script.h>
#include <torch/torch.h>

class DLInterface;


class DLInterface : public art::EDProducer {
public:
  explicit DLInterface(fhicl::ParameterSet const & p);
  // The compiler-generated destructor is fine for non-base
  // classes without bare pointers or other resource use.

  // Plugins should not be copied or assigned.
  DLInterface(DLInterface const &) = delete;
  DLInterface(DLInterface &&) = delete;
  DLInterface & operator = (DLInterface const &) = delete;
  DLInterface & operator = (DLInterface &&) = delete;

  // Required functions.
  void produce(art::Event & e) override;
  void beginJob() override;
  
private:

  // Declare member data here.
  larcv::SuperaWire      _imagemaker;
  supera::ImageMetaMaker _metamaker;
  //larcv::SuperaMetaMaker _metamaker;
  larcv::UBSplitDetector _imagesplitter;

  std::string _wire_producer_name;
  std::string _supera_config;
  std::string _pytorch_net_script;

  //zmq::context_t* _context;
  //zmq::socket_t*  _socket; 

};


DLInterface::DLInterface(fhicl::ParameterSet const & p)
// :
// Initialize member data here.
{
  // Call appropriate produces<>() functions here.
  _wire_producer_name = p.get<std::string>("WireProducerName");
  _supera_config      = p.get<std::string>("SuperaConfigFile");

  std::string supera_cfg;
  cet::search_path finder("FHICL_FILE_PATH");
  if( !finder.find_file(_supera_config, supera_cfg) )
    throw cet::exception("DLInterface") << "Unable to find supera cfg in "  << finder.to_string() << "\n";

  // check cfg content top level
  larcv::PSet main_cfg = larcv::CreatePSetFromFile(_supera_config).get<larcv::PSet>("DLInterface");

  // get list of processors
  std::vector<std::string> process_names = main_cfg.get< std::vector<std::string> >("ProcessName");
  std::vector<std::string> process_types = main_cfg.get< std::vector<std::string> >("ProcessType");
  larcv::PSet              process_list  = main_cfg.get<larcv::PSet>("ProcessList");
  larcv::PSet              split_cfg     = main_cfg.get<larcv::PSet>("UBSplitConfig"); 
  
  if ( process_names.size()!=process_types.size() ) {
    throw std::runtime_error( "Number of Supera ProcessName(s) and ProcessTypes(s) is not the same." );
  }

  bool cfgmeta = false;
  bool cfgwire = false;
  for (int i=0; i<(int)process_names.size(); i++) {
    std::string name = process_names[i];
    std::string type = process_types[i];

    if ( !process_list.contains_pset(name) ) {
      std::cout << "Not in ProcessList: " << name << "/" << type << std::endl;
      continue;
    }

    if ( type=="SuperaWire" ) {
      cfgwire = true;
      _imagemaker.configure( process_list.get<larcv::PSet>(name) );
    }
    else if ( type=="ImageMetaMaker" ) {
      cfgmeta = true;
      _metamaker.configure( process_list.get<larcv::PSet>(name) );
    }
  }  
  
  if ( !cfgmeta || !cfgwire ) {
    throw std::runtime_error("DLInterface_module needs a configuration for an instance of ImageMetaMaker and SuperaWire");
  }

  // configure image splitter
  _imagesplitter.configure( split_cfg );

  _pytorch_net_script = p.get<std::string>("PyTorchNetScript");

  // open the zmq socket
  //_context = new zmq::context_t(1);
  //_socket  = new zmq::socket_t( *_context, ZMQ_REQ );
  //char identity[10];
  //sprintf(identity,"larbys00");
  //_socket->setsockopt( ZMQ_IDENTITY, identity, 8 );
  //_socket->connect("tcp://localhost:5559");
}

void DLInterface::produce(art::Event & e)
{

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

  //
  // define the image meta
  //
  const std::vector<larcv::ImageMeta>& meta_v = _metamaker.Meta();
  //const std::vector<larcv::ImageMeta>& meta_v = _imagemaker.Meta();

  //
  // execute wire2image (get the full image)
  //
  std::vector<larcv::Image2D> image_v = supera::Wire2Image2D(meta_v, *data_h, _imagemaker.TimeOffset() );
  std::cout << "DLInterface: images made" << std::endl;
  for (auto& img : image_v ) {
    std::cout << img.meta().dump() << std::endl;
  }

  //
  // compress
  //
  for(size_t plane=0; plane<image_v.size(); ++plane) {
    auto& image = image_v[plane];
    image.compress(image.meta().rows() / _metamaker.RowCompressionFactor().at(plane),
		   image.meta().cols() / _metamaker.ColCompressionFactor().at(plane),
		   larcv::Image2D::kSum);
  }
  
  
  //
  // split image into subregions
  //
  std::vector<larcv::Image2D> splitimg_v;
  std::vector<larcv::ROI>     splitroi_v;
  _imagesplitter.process( image_v, splitimg_v, splitroi_v );

  //
  // form the message
  //
  std::cout << "Number of split images: " << splitimg_v.size() << std::endl;
  std::cout << "Number of split roi: " << splitroi_v.size() << std::endl;  
  
  int nimgs = splitimg_v.size()/image_v.size(); // number of images / number of planes
  int nplanes = image_v.size();

  //debug
  nimgs   = 1;
  nplanes = 1;

  std::vector<std::string> msg_meta_v(nimgs*nplanes);
  std::vector<std::string> msg_name_v(nimgs*nplanes);
  std::vector< std::vector<unsigned char> > msg_img_v(nimgs*nplanes);

  int img_msg_totsize = 0;

 
  for (int iimg=0; iimg<nimgs; iimg++ ) {
    //larcv::ROI& roi = splitroi_v[iimg];
    for (int p=0; p<nplanes; p++) {
      larcv::Image2D& img = splitimg_v[ iimg*image_v.size() + p ];

      char zname[50];
      sprintf( zname, "croicropped_p%d_iimg%d", p, iimg );

      std::string msg_name = zname;
      std::string msg_meta = img.meta().dump();

      // TMessage msg_img;
      // msg_img.WriteObject( &img );
      // std::string str_msg_img = msg_img.Buffer();
      std::vector<unsigned char> data;
      //zpp::serializer::memory_output_archive out(data);
      //out( img );

      img_msg_totsize += data.size();
      
      msg_name_v[iimg*image_v.size()+p] = msg_name;
      msg_meta_v[iimg*image_v.size()+p] = msg_meta;
      msg_img_v[iimg*image_v.size()+p]  = data;
    }
  }

  std::cout << "Created " << msg_img_v.size() << " messages. Total image message size: " << img_msg_totsize << " (chars)" << std::endl;

  std::shared_ptr<torch::jit::script::Module> module = torch::jit::load( _pytorch_net_script );
  if ( module==nullptr )
    throw cet::exception("DLInterface") << "Could not load model from " << _pytorch_net_script << std::endl;
  std::cout << "Network Loaded" << std::endl;

  std::vector<torch::jit::IValue> inputs[3];
  for (int iimg=0; iimg<nimgs; iimg++ ) {
    //larcv::ROI& roi = splitroi_v[iimg];
    for (int p=0; p<nplanes; p++) {
      larcv::Image2D& img = splitimg_v[ iimg*image_v.size() + p ];
      inputs[p].push_back( larcv::torchutils::as_tensor( img ) );
    }
    break;
  }

  // run the net!
  at::Tensor output = module->forward(inputs[2]).toTensor();
  std::cout << "network produced: " << output.size(0) << "," << output.size(1) << "," << output.size(2) << std::endl;
  
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
  
}

void DLInterface::beginJob()
{
  _imagemaker.initialize();
}


DEFINE_ART_MODULE(DLInterface)
