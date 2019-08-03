////////////////////////////////////////////////////////////////////////
// Class:       DLInterface
// Plugin Type: producer (art v2_05_01)
// File:        DLInterface_module.cc
//
// Generated at Wed Aug 29 05:49:09 2018 by Taritree,,, using cetskelgen
// from cetlib version v1_21_00.
////////////////////////////////////////////////////////////////////////

#include "Python.h"

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

// larsoft
#include "lardataobj/RecoBase/OpFlash.h"
#include "larevt/CalibrationDBI/Interface/ChannelStatusService.h"
#include "larevt/CalibrationDBI/Interface/ChannelStatusProvider.h"
#include "larcore/Geometry/Geometry.h"

// ubcv
#include "ubcv/LArCVImageMaker/LArCVSuperaDriver.h"
#include "ubcv/LArCVImageMaker/ImageMetaMaker.h"
#include "ubcv/LArCVImageMaker/SuperaMetaMaker.h"
#include "ubcv/LArCVImageMaker/SuperaWire.h"
#include "ubcv/LArCVImageMaker/LAr2Image.h"
#include "ubcv/ubdldata/pixeldata.h"

// larcv
#include "larcv/core/Base/larcv_base.h"
#include "larcv/core/Base/PSet.h"
#include "larcv/core/Base/LArCVBaseUtilFunc.h"
#include "larcv/core/DataFormat/EventImage2D.h"
#include "larcv/core/DataFormat/Image2D.h"
#include "larcv/core/DataFormat/EventSparseImage.h"
#include "larcv/core/DataFormat/SparseImage.h"
#include "larcv/core/DataFormat/EventClusterMask.h"
#include "larcv/core/DataFormat/ClusterMask.h"
#include "larcv/core/DataFormat/ImageMeta.h"
#include "larcv/core/DataFormat/ROI.h"
#include "larcv/core/json/json_utils.h"

// larlite
#include "DataFormat/storage_manager.h"
#include "DataFormat/opflash.h"
#include "DataFormat/larflow3dhit.h"

// ublarcvapp
#include "ublarcvapp/UBImageMod/UBSplitDetector.h"
#include "ublarcvapp/ubdllee/FixedCROIFromFlashAlgo.h"
#include "ublarcvapp/ubdllee/FixedCROIFromFlashConfig.h"
#include "ublarcvapp/UBImageMod/InfillDataCropper.h"
#include "ublarcvapp/UBImageMod/InfillImageStitcher.h"

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

// dlintegration modules
#include "ServerInterface.h"   // base class for server interfaces
#include "SSNet.h"             // not currently used
#include "LArFlow.h"           // server interface for larflow
#include "Infill.h"            // server interface for infill
#include "CosmicMRCNN.h"       // server interface for cosmic mask-rcnn
#include "PyNet_CosmicMRCNN.h" // cpu python interface for cosmic mask-rcnn
#include "PyNet_SparseInfill.h"      // cpu python interface for infill

#endif

class DLInterface;


class DLInterface : public art::EDProducer, larcv::larcv_base {
public:
  explicit DLInterface(fhicl::ParameterSet const & p);
  // The compiler-generated destructor is fine for non-base
  // classes without bare pointers or other resource use.

  typedef enum { kDoNotRun=0, kServer, kPyTorchCPU, kTensorFlowCPU, kDummyServer } NetInterface_t;
  typedef enum { kWholeImageSplitter=0, kFlashCROI } Cropper_t;
  typedef enum { kSSNET=0, kLARFLOW, kINFILL, kUBMRCNN, kNUM_NETS } DLNetworks_t;

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
  int _verbosity;

  // supera/image formation
  larcv::LArCVSuperaDriver _supera;  
  std::string _wire_producer_name;
  std::string _supera_config;
  int runSupera( art::Event& e, 
		 std::vector<larcv::Image2D>& wholeview_imgs,
		 larcv::EventChStatus** ev_chstatus );

  // storage for larlite
  larlite::storage_manager _out_larlite;

  // image processing/splitting/cropping
  bool _make_wholeimg_crops;
  bool _make_flashroi_crops;
  Cropper_t _ssnet_crop_method;
  ublarcvapp::UBSplitDetector _imagesplitter; //< full splitter
  ublarcvapp::ubdllee::FixedCROIFromFlashAlgo _croifromflashalgo; //< croi from flash
  std::string _ubcrop_trueflow_cfg; //< config for the ssnet/larflow splitting and cropping algorithm
  std::string _infill_split_cfg;    //< config for the infill splitting and cropping algorithm
  bool        _save_detsplit_input;
  bool        _save_detsplit_output;
  std::string _opflash_producer_name;
  std::vector<larcv::Image2D> _splitimg_v; //< container holding split images
  std::vector<larcv::Image2D> _flashroi_v; //< container holding split images
  int runWholeViewSplitter( const std::vector<larcv::Image2D>& wholeview_v, 
			    std::vector<larcv::Image2D>& splitimg_v,
			    std::vector<larcv::ROI>&     splitroi_v );
  int runCROIfromFlash( const std::vector<larcv::Image2D>& wholeview_v, 
			art::Event& e,
			std::vector<larcv::Image2D>& splitimg_v,
			std::vector<larcv::ROI>&     splitroi_v );
  
  // the network interface to run
  NetInterface_t _ssnet_mode;
  NetInterface_t _larflow_mode;
  NetInterface_t _infill_mode;
  NetInterface_t _cosmic_mrcnn_mode;
  bool _connect_to_server;

  // interface: pytorch cpu
  std::vector<std::string> _pytorch_net_script; // one for each plane
#ifdef HAS_TORCH
  std::vector< std::shared_ptr<torch::jit::script::Module> > _module_ubssnet;  //< pointer to pytorch network
#endif
  void loadNetwork_PyTorchCPU();
  int runPyTorchCPU( const std::vector<larcv::Image2D>& wholeview_v,
		     std::vector<larcv::Image2D>& splitimg_v, 
		     std::vector<larcv::ROI>& splitroi_v, 
		     std::vector<larcv::Image2D>& showerout_v,
		     std::vector<larcv::Image2D>& trackout_v);

  

  // interface: server (common)
  std::string      _broker_address;
  std::string      _broker_port;
  int              _client_timeout_secs;
  int              _request_max_tries;
  int              _max_n_broker_reconnect;
  zmq::context_t*  _context;

  zmq::socket_t*   _socket[4]; 
  zmq::pollitem_t* _poller[4];

  // server modules
  dl::LArFlow*     _larflow_server;
  dl::Infill*      _infill_server;
  dl::CosmicMRCNN* _ubmrcnn_server;

  // python script modules
  std::vector< std::string > _ubmrcnn_weight_file_v;
  std::vector< std::string > _ubmrcnn_config_file_v;
  ubcv::ubdlintegration::PyNetCosmicMRCNN* _ubmrcnn_script;

  std::vector< std::string > _sparseinfill_weight_file_v;
  ubcv::ubdlintegration::PyNetSparseInfill* _sparseinfill_script;
  
  void openServerSocket();
  void closeServerSocket();
  size_t serializeImages( const size_t nplanes,
			  const std::vector<larcv::Image2D>& img_v,   
			  std::vector< std::vector<unsigned char> >& msg_img_v,
			  std::vector<std::string>& msg_meta_v,
			  std::vector<std::string>& msg_name_v );


  // interface: ssnet gpu server
  int runSSNetServer( const int run, const int subrun, const int event,
		      const std::vector<larcv::Image2D>& wholeview_v, 
		      std::vector<larcv::Image2D>& splitimg_v, 
		      std::vector<larcv::ROI>& splitroi_v, 
		      std::vector<larcv::Image2D>& showerout_v,
		      std::vector<larcv::Image2D>& trackout_v );

  // interface: larflow gpu server
  int runLArFlowServer( const int run, const int subrun, const int event,
			const std::vector<larcv::Image2D>& wholeview_v, 
			std::vector<larcv::Image2D>& splitimg_v, 
			std::vector<larcv::ROI>& splitroi_v, 
			std::vector<larcv::SparseImage>& larflow_results_v,
			std::vector<larlite::larflow3dhit>& larflow_hit_v );

  // interface: infill gpu server
  int runInfillServer( const int run, const int subrun, const int event, 
		       std::vector<std::vector<larcv::SparseImage> >& adc_crops_vv,
		       std::vector<std::vector<larcv::SparseImage> >& results_vv,
		       const float threshold );
  // interface: infill cpu
  int runInfill_cpu( const int run, const int subrun, const int event, 
		     std::vector<std::vector<larcv::SparseImage> >& adc_crops_vv,
		     std::vector<std::vector<larcv::SparseImage> >& results_vv );
  

  // interface: run cosmic mrcnn server
  int runCosmicMRCNNServer( const int run, const int subrun, const int event, 
			    const std::vector<larcv::Image2D>& wholeview_v, 
			    std::vector<std::vector<larcv::ClusterMask> >& results_vv );
  int runCosmicMRCNN_cpu( const int run, const int subrun, const int event, 
			  const std::vector<larcv::Image2D>& wholeview_v, 
			  std::vector<std::vector<larcv::ClusterMask> >& results_vv );

  
  // interface: dummy server (for debug)
  int runDummyServer( art::Event& e );
  void sendDummyMessage();
  size_t sendSSNetPlaneImages( const int plane,
			       const std::vector<int>& reply_ok,
			       std::vector< zmq::message_t >& zmsg_v,
			       bool debug );


  // merger/post-processing
  void mergeSSNetOutput( const std::vector<larcv::Image2D>& wholeview_v, 
			 const std::vector<larcv::ROI>& splitroi_v, 
			 const std::vector<larcv::Image2D>& showerout_v,
			 const std::vector<larcv::Image2D>& trackout_v,
			 std::vector<larcv::Image2D>& showermerged_v,
			 std::vector<larcv::Image2D>& trackmerged_v );

  // pre-processing: infill
  void splitImageForInfill( const std::vector<larcv::Image2D>& wholeview_v, 
			    larcv::EventChStatus& ev_chstatus,
			    std::vector<larcv::Image2D>& label_v,
			    std::vector<std::vector<larcv::SparseImage> >& adc_crops_vv,
			    std::vector<std::vector<larcv::Image2D> >& label_crops_vv,
			    const float threshold,
			    const std::string infill_split_cfg );

  // post-processing: infill
  void stitchSparseCrops( const std::vector< std::vector<larcv::SparseImage> >& netout_vv,
			  const std::vector<larcv::Image2D>& wholeview_adc_v,
			  larcv::EventChStatus& ev_chstatus,
			  std::vector< larcv::Image2D >& mergedout_v );


  /// save to art::Event
  void saveSSNetArtProducts( art::Event& ev,
			     const std::vector<larcv::Image2D>& wholeimg_v,
			     const std::vector<larcv::Image2D>& showermerged_v,
			     const std::vector<larcv::Image2D>& trackmerged_v );
  std::vector<float> _pixelthresholds_forsavedscores;

  void saveLArFlowArtProducts( art::Event& ev, 
			       const std::vector<larcv::Image2D>& wholeimg_v,
			       const std::vector<larcv::SparseImage>& larflow_results_v,
			       std::vector<larlite::larflow3dhit>& larflow_hit_v,
			       const std::vector<larcv::Image2D>& showermerged_v,
			       const std::vector<larcv::Image2D>& trackmerged_v );

  void saveInfillArtProducts( art::Event& ev, 
			      const std::vector<larcv::Image2D>& infill_stitched_v,
			      const std::vector<larcv::Image2D>& infill_label_v );

  void saveCosmicMRCNNArtProducts( art::Event& ev,
				   const std::vector<larcv::Image2D>& wholeview_adc_v,
				   const std::vector< std::vector<larcv::ClusterMask> >& ubmrcnn_result_vv );
  

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
  produces< std::vector<ubdldata::pixeldata> >( "ssnet" );
  produces< std::vector<ubdldata::pixeldata> >( "larflow" );
  produces< std::vector<ubdldata::pixeldata> >( "larflowhits" );
  produces< std::vector<ubdldata::pixeldata> >( "infill" );
  produces< std::vector<ubdldata::pixeldata> >( "ubmrcnn" );

  // read in parameters and configure

  // product to get recob::wire data from
  _wire_producer_name = p.get<std::string>("WireProducerName");

  // name of supera configuration file
  _supera_config      = p.get<std::string>("SuperaConfigFile");


  // larlite output file
  _out_larlite.set_io_mode( larlite::storage_manager::kWRITE );
  _out_larlite.set_out_filename( p.get<std::string>("larliteOutputFile") );

  // interace to network(s)
  size_t nnets = 4;
  std::vector<string> networks_v;
  networks_v.push_back("SSNet");
  networks_v.push_back("LArFlow");
  networks_v.push_back("Infill");
  networks_v.push_back("CosmicMRCNN");
  NetInterface_t* mode_v[4] = { &_ssnet_mode, 
				&_larflow_mode,
				&_infill_mode,
				&_cosmic_mrcnn_mode};
  
  _connect_to_server = false;
  for ( size_t inet=0; inet<nnets; inet++ ) {
    std::string netname = networks_v.at(inet);
    std::string inter   = p.get<std::string>(netname+"Mode","DoNotRun");
    if ( inter=="DoNotRun" )
      *mode_v[inet] = kDoNotRun;
    else if ( inter=="Server" ) {
      *mode_v[inet] = kServer;
      _connect_to_server = true;
    }
    else if ( inter=="PyTorchCPU" )
      *mode_v[inet] = kPyTorchCPU;
    else if ( inter=="TensorFlowCPU" ) {
      *mode_v[inet] = kTensorFlowCPU;
      throw cet::exception("DLInterface") << "TensorFlowCPU interface not yet implemented" << std::endl;
    }
    else if ( inter=="DummyServer" ) // for debugging
      *mode_v[inet] = kDummyServer;
    else {
      throw cet::exception("DLInterface") << "unrecognized network interface, " << inter << ". "
					  << "choices: { DoNotRun, Server, PytorchCPU, TensorFlowCPU, DummyServer }"
					  << std::endl;
    }
    LARCV_INFO() << "Network[ " << netname << " ] mode=" << inter << std::endl;
  }

  // =================================
  //  INTERFACE SPECIFIC CONFIGS
  //  --------------------------  


  // If we are connecting to the server, get the parameters
  if ( _connect_to_server ) {
    auto const server_pset = p.get<fhicl::ParameterSet>("ServerConfiguration");
    _broker_address = server_pset.get<std::string>("BrokerAddress");
    _broker_port    = server_pset.get<std::string>("BrokerPort");
    _client_timeout_secs    = server_pset.get<int>("RequestTimeoutSecs",60);
    _request_max_tries      = server_pset.get<int>("RequestMaxTries",3);
    _max_n_broker_reconnect = server_pset.get<int>("MaxBrokerReconnects",3);
  }

  // =================================
  // Image cropping/splitting config
  // -------------------------------
  _make_wholeimg_crops = p.get<bool>("DoWholeImageCrop",true);
  _make_flashroi_crops = p.get<bool>("DoFlashROICrop",true);

  std::string ssnet_crop_method = p.get<std::string>("SSNetCropMethod","FlashCROI");
  if ( ssnet_crop_method=="FlashCROI" )
    _ssnet_crop_method = kFlashCROI;
  else if ( ssnet_crop_method=="WholeImageSplitter" )
    _ssnet_crop_method = kWholeImageSplitter;
  else {
    throw cet::exception("DLInterface") << "unrecognized crop method for SSNet, " << ssnet_crop_method << ". "
					<< "choices: { FlashCROI, WholeImageSplitter }"
					<< std::endl;
  }
  // override whole image splitter is ssnet requires it
  if ( _ssnet_crop_method==kWholeImageSplitter ) {
    _make_wholeimg_crops = true;
  }


  // ubsplitter config: for larflow
  cet::search_path split_finder("FHICL_FILE_PATH");
  std::string splitter_cfg = p.get<std::string>("UBCropConfig","ubcrop.cfg");
  if( !split_finder.find_file(splitter_cfg,_ubcrop_trueflow_cfg) )
    throw cet::exception("DLInterface") 
      << "Unable to find UBSplitDet config "
      << "(" << splitter_cfg << ") -> "
      << "(" << _ubcrop_trueflow_cfg << ") "
      << " within paths: "  << split_finder.to_string() << "\n";
  else
    LARCV_DEBUG() << "UBSplitDetector(ssnet/larflow) config path: " << _ubcrop_trueflow_cfg << std::endl;

  if ( _make_wholeimg_crops ) {
    _save_detsplit_input  = p.get<bool>("SaveDetsplitImagesInput",false);
    _save_detsplit_output = p.get<bool>("SaveNetOutSplitImagesOutput",false);
  }

  if ( _make_flashroi_crops ) {
    _opflash_producer_name = p.get<std::string>("OpFlashProducerName","SimpleFlashBeam");
  }

  // =================================
  // UB-MRCNN
  // -------------------------------
  if ( *mode_v[kUBMRCNN] == kPyTorchCPU ) {
    _ubmrcnn_weight_file_v = p.get<std::vector<std::string> >("CosmicMRCNN_WeightFiles");
    _ubmrcnn_config_file_v = p.get<std::vector<std::string> >("CosmicMRCNN_ConfigFiles");
  }

  // =================================
  // INFILL
  // -------------------------------
  if ( *mode_v[kINFILL] == kPyTorchCPU ) {
    _sparseinfill_weight_file_v = p.get<std::vector<std::string> >("SparseInfill_WeightFiles");
  }
  
  // ubsplitter config: for infill
  cet::search_path infill_split_finder("FHICL_FILE_PATH");
  std::string infill_split_cfg = p.get<std::string>("InfillCropConfig","infill_split.cfg");
  if( !infill_split_finder.find_file(infill_split_cfg,_infill_split_cfg) )
    throw cet::exception("DLInterface") 
      << "Unable to find UBSplitDet config for Infill "
      << "(" << infill_split_cfg << ") -> "
      << "(" << _infill_split_cfg << ") "
      << " within paths: "  << infill_split_finder.to_string() << "\n";
  else
    LARCV_DEBUG() << "UBSplitDetector(infill) config path: " << _infill_split_cfg << std::endl;


  // =================================
  // Supera configuration
  // -------------------------------

  std::string supera_cfg;
  cet::search_path finder("FHICL_FILE_PATH");
  if( !finder.find_file(_supera_config, supera_cfg) )
    throw cet::exception("DLInterface") << "Unable to find supera cfg in "  << finder.to_string() << "\n";
  std::cout << "LOADING supera config: " << supera_cfg << std::endl;

  // check cfg content top level
  larcv::PSet main_cfg = larcv::CreatePSetFromFile(supera_cfg).get<larcv::PSet>("ProcessDriver");

  // get list of processors
  std::vector<std::string> process_names = main_cfg.get< std::vector<std::string> >("ProcessName");
  std::vector<std::string> process_types = main_cfg.get< std::vector<std::string> >("ProcessType");
  larcv::PSet              process_list  = main_cfg.get<larcv::PSet>("ProcessList");
  larcv::PSet              split_cfg     = main_cfg.get<larcv::PSet>("UBSplitConfig"); 
  
  if ( process_names.size()!=process_types.size() ) {
    throw std::runtime_error( "Number of Supera ProcessName(s) and ProcessTypes(s) is not the same." );
  }

  // configure supera (convert art::Event::CalMod -> Image2D)
  _supera.configure(supera_cfg);
  _supera.driver().random_access(false);

  // configure image splitter (fullimage into detsplit image)
  _imagesplitter.configure( split_cfg );

  // get the path to the saved ssnet
  _pytorch_net_script = p.get< std::vector<std::string> >("PyTorchNetScript");

  // configuration for art product output
  _pixelthresholds_forsavedscores = p.get< std::vector<float> >("PixelThresholdsForSavedScoresPerPlane");

  // verbosity
  _verbosity = p.get<int>("Verbosity",2);
  set_verbosity( (::larcv::msg::Level_t) _verbosity );
  _supera.set_verbosity( (::larcv::msg::Level_t) _verbosity );

}

/** 
 * produce DL network products
 *
 */
void DLInterface::produce(art::Event & e)
{

  // ===============================================
  // get larcv products from supers
  //   * wholeview images for the network
  //   * EventChStatus
  // ----------------------------------------
  std::vector<larcv::Image2D> wholeview_v;
  larcv::EventChStatus* ev_chstatus = nullptr;
  int nwholeview_imgs = runSupera( e, wholeview_v, &ev_chstatus );
  LARCV_INFO() << "number of wholeview images: " << nwholeview_imgs << std::endl;

  // ===============================================
  // cropping
  // ----------------------------------------

  // we often have to pre-process the image, e.g. split it.
  // eventually have options here. But for now, wholeview splitter
  std::vector<larcv::Image2D> splitimg_v; // for whole-view splits
  std::vector<larcv::ROI>     splitroi_v; // for whole-view splits
  std::vector<larcv::Image2D> croiimg_v;  // for croi images
  std::vector<larcv::ROI>     croi_v;     // for croi images
  
  int nsplit_imgs = 0;
  int ncroi_imgs  = 0;
  if ( _make_wholeimg_crops )
    nsplit_imgs = runWholeViewSplitter( wholeview_v, splitimg_v, splitroi_v );
  if ( _make_flashroi_crops ) 
    ncroi_imgs = runCROIfromFlash( wholeview_v, e, croiimg_v, croi_v );

  LARCV_INFO() << "number of whole-image crops: " << nsplit_imgs << std::endl;
  LARCV_INFO() << "number of croi crops: " << ncroi_imgs << std::endl;
  
  // ==========================================
  // run networks (or not)

  // -------
  // ssnet
  // -------
  int ssnet_status = 0;

  // ssnet output containers
  std::vector<larcv::Image2D> showerout_v; // crop scores
  std::vector<larcv::Image2D> trackout_v;  // crop scores
  std::vector<larcv::Image2D> showermerged_v; // merged scores
  std::vector<larcv::Image2D> trackmerged_v;  // merged scores

  // choose ssnet input
  std::vector<larcv::Image2D>* ssnet_input 
    = ( _ssnet_crop_method==kFlashCROI ) ? &croiimg_v : &splitimg_v ;
  std::vector<larcv::ROI>* ssnet_roi
    = ( _ssnet_crop_method==kFlashCROI ) ? &croi_v : &splitroi_v ;

  if ( ssnet_input->size() && _ssnet_mode!=kDoNotRun ) {
    LARCV_INFO() << "Run SSNet" << std::endl;

    switch (_ssnet_mode) {
    case kDummyServer:
    case kTensorFlowCPU:
      ssnet_status = runDummyServer(e);
      break;
    case kPyTorchCPU:
      ssnet_status = runPyTorchCPU( wholeview_v, *ssnet_input, *ssnet_roi, showerout_v, trackout_v );
      break;
    case kServer:
      ssnet_status = runSSNetServer( e.id().run(), e.id().subRun(), e.id().event(),
				     wholeview_v, *ssnet_input, *ssnet_roi, showerout_v, trackout_v );
      break;
    case kDoNotRun:
    default:
      throw cet::exception("DLInterface") << "Unrecognized ssnet mode(" << _ssnet_mode << ")" << std::endl;
      break;
    }
    if ( ssnet_status!=0 )
      throw cet::exception("DLInterface") << "Error running SSNet network" << std::endl;

    // merge the output
    mergeSSNetOutput( wholeview_v, *ssnet_roi, showerout_v, trackout_v, showermerged_v, trackmerged_v );

  }// if SSNET run

  if ( !_save_detsplit_output ) {
    showerout_v.clear();
    trackout_v.clear();
  }

  // produce the art data product
  saveSSNetArtProducts( e, wholeview_v, showermerged_v, trackmerged_v );


  // -------------------------------------------------
  // LArFlow
  // -------------------------------------------------
  int larflow_status = 0;
  std::vector<larcv::SparseImage> larflow_results_v; // larflow output for cropped images
  std::vector<larlite::larflow3dhit> larflow_hit_v; // larflow 3d hits

  if ( wholeview_v.size()>0 && _larflow_mode!=kDoNotRun ) {

    LARCV_INFO() << "Run LArFlow" << std::endl;
    
    switch (_larflow_mode) {
    case kServer:
      larflow_status = runLArFlowServer(e.id().run(), e.id().subRun(), e.id().event(),
					wholeview_v, splitimg_v, splitroi_v, 
					larflow_results_v, larflow_hit_v );
      break;
    case kDoNotRun:
    case kDummyServer:
    case kTensorFlowCPU:
    case kPyTorchCPU:
    default:
      throw cet::exception("DLInterface") 
	<< "Attempting to run larflow network in unimplemented mode "
	<< "(" << _larflow_mode << ")." << std::endl;      
      break;
    }
    if ( larflow_status!= 0 ) {
      throw cet::exception("DLInterface")
	<< "Error running LArFlow Network" << std::endl;
    }
    
    
  } // if LARFLOW run
    
  // produce the art data product
  saveLArFlowArtProducts( e, wholeview_v, larflow_results_v, larflow_hit_v, 
			  showermerged_v, trackmerged_v );

  // clear splitimg container if we're not going to save it
  if ( !_save_detsplit_input )
    splitimg_v.clear();


  // ----------------------------------------------
  // infill
  // ----------------------------------------------
  int infill_status = 0;
  std::vector< std::vector<larcv::SparseImage> > infill_crop_vv;   // adc image, cropped, sparse form
  std::vector< std::vector<larcv::Image2D> >     infill_label_vv;  // label image, cropped, sparse form
  std::vector< std::vector<larcv::SparseImage> > infill_netout_vv; // infill net output, cropped, sparse form
  std::vector< larcv::Image2D > in_label_v;                        // label image, whole view
  std::vector< larcv::Image2D > infill_merged_out_v;               // merged infill output
  if ( wholeview_v.size() && _infill_mode!=kDoNotRun ) {

    LARCV_INFO() << "Run Infill" << std::endl;
    splitImageForInfill( wholeview_v, *ev_chstatus,
			 in_label_v,
			 infill_crop_vv, 
			 infill_label_vv,
			 10.0,
			 _infill_split_cfg );

    switch (_infill_mode) {
    case kServer:
      infill_status = runInfillServer( e.id().run(), e.id().subRun(), e.id().event(),
				       infill_crop_vv, 
				       infill_netout_vv,
				       10.0 );
      break;
    case kPyTorchCPU:
      infill_status = runInfill_cpu( e.id().run(), e.id().subRun(), e.id().event(),
				     infill_crop_vv, infill_netout_vv  );
      break;
    case kDoNotRun:
    case kDummyServer:
    case kTensorFlowCPU:
    default:
      throw cet::exception("DLInterface") 
	<< "Attempting to run infill network in unimplemented mode "
	<< "(" << _infill_mode << ")." << std::endl;      
      break;
    }
    if ( infill_status!= 0 ) {
      throw cet::exception("DLInterface")
	<< "Error running Infill Network" << std::endl;
    }

    stitchSparseCrops( infill_netout_vv,
		       wholeview_v,
		       *ev_chstatus,
		       infill_merged_out_v );
  }

  // clear infill crops: not needed unless for debug downstream
  infill_crop_vv.clear();
  infill_label_vv.clear();
  infill_netout_vv.clear();

  saveInfillArtProducts( e, infill_merged_out_v, in_label_v );

  // ----------------------------------------------
  // mask RCNN
  // ----------------------------------------------
  int ubmrcnn_status = 0;
  std::vector< std::vector<larcv::ClusterMask> > ubmrcnn_result_vv(wholeview_v.size());
  if ( wholeview_v.size() && _cosmic_mrcnn_mode!=kDoNotRun ) {

    LARCV_INFO() << "Run Cosmic Mask-RCNN" << std::endl;

    switch (_cosmic_mrcnn_mode) {
    case kServer:
      ubmrcnn_status = runCosmicMRCNNServer( e.id().run(), e.id().subRun(), e.id().event(),
					     wholeview_v, 
					     ubmrcnn_result_vv );
      break;
    case kPyTorchCPU:
      ubmrcnn_status = runCosmicMRCNN_cpu( e.id().run(), e.id().subRun(), e.id().event(),
					   wholeview_v,
					   ubmrcnn_result_vv );
      break;
    case kDoNotRun:
    case kDummyServer:
    case kTensorFlowCPU:
    default:
      throw cet::exception("DLInterface") 
	<< "Attempting to run Cosmic Mask RCNN network in unimplemented mode "
	<< "(" << _cosmic_mrcnn_mode << ")." << std::endl;
      break;
    }
    if ( ubmrcnn_status!= 0 ) {
      throw cet::exception("DLInterface")
	<< "Error running Cosmic Mask RCNN Network" << std::endl;
    }    
  }
  
  saveCosmicMRCNNArtProducts( e, wholeview_v, ubmrcnn_result_vv );



  // ================================================================
  // save LArCV output
  // ----------------------------------------------------------------


  // wholeview and cropped output
  // -------------------------------------

  // LArCV iomanager
  larcv::IOManager& io = _supera.driver().io_mutable();
  // save the wholeview images back to the supera IO
  larcv::EventImage2D* ev_imgs  = (larcv::EventImage2D*) io.get_data( larcv::kProductImage2D, "wire" );
  //std::cout << "wire eventimage2d=" << ev_imgs << std::endl;
  ev_imgs->Emplace( std::move(wholeview_v) );

  // save detsplit input
  larcv::EventImage2D* ev_splitdet = nullptr;
  if ( _save_detsplit_input ) {
    ev_splitdet = (larcv::EventImage2D*) io.get_data( larcv::kProductImage2D, "detsplit" );
    ev_splitdet->Emplace( std::move(splitimg_v) );
  }

  // ssnet
  // -------

  // save ssnet detsplit output
  larcv::EventImage2D* ev_netout_split[2] = {nullptr,nullptr};
  if ( _save_detsplit_output ) {
    ev_netout_split[0] = (larcv::EventImage2D*) io.get_data( larcv::kProductImage2D, "netoutsplit_shower" );
    ev_netout_split[1] = (larcv::EventImage2D*) io.get_data( larcv::kProductImage2D, "netoutsplit_track" );
    ev_netout_split[0]->Emplace( std::move(showerout_v) );
    ev_netout_split[1]->Emplace( std::move(trackout_v) );
  }
  
  // save ssnet merged output larcv images
  larcv::EventImage2D* ev_merged[2] = {nullptr,nullptr};
  ev_merged[0] = (larcv::EventImage2D*)io.get_data( larcv::kProductImage2D, "ssnetshower" );
  ev_merged[1] = (larcv::EventImage2D*)io.get_data( larcv::kProductImage2D, "ssnettrack" );
  ev_merged[0]->Emplace( std::move(showermerged_v) );
  ev_merged[1]->Emplace( std::move(trackmerged_v) );
  
  // larflow
  // --------

  // save larflow output images
  larcv::EventSparseImage* ev_larflowout 
    = (larcv::EventSparseImage*)io.get_data( larcv::kProductSparseImage, "larflow" );
  ev_larflowout->Move( larflow_results_v );
  
  // save larflow hits into larlite output
  larlite::event_larflow3dhit* ev_larflow3dhit 
    = (larlite::event_larflow3dhit*)_out_larlite.get_data( larlite::data::kLArFlow3DHit, "flowhits" );
  for ( auto& hit : larflow_hit_v )
    ev_larflow3dhit->emplace_back( std::move(hit) );

  // infill
  // -------

  // for debug
  //  larcv::EventImage2D* ev_infillcropout[3]  = { nullptr, nullptr, nullptr };
  //larcv::EventImage2D* ev_infill_labelout[3] = { nullptr, nullptr, nullptr };
  // for ( size_t p=0; p<3; p++ ) {
  //   char treename[64];
  //   sprintf( treename, "infillcrop_plane%d", (int)p );
  //   ev_infillcropout[p] = (larcv::EventImage2D*)io.get_data( larcv::kProductImage2D, treename );
  //   if ( p<infill_netout_vv.size() ) {
  //     for ( auto& spimg : infill_netout_vv.at(p) ) {
  // 	ev_infillcropout[p]->Append( spimg.as_Image2D().at(0) );
  //     }
  //   }

  //   char labeltreename[64];
  //   sprintf( labeltreename, "labelcrop_plane%d", (int)p );
  //   ev_infill_labelout[p] = (larcv::EventImage2D*)io.get_data( larcv::kProductImage2D, labeltreename );
  //   if ( p<infill_label_vv.size() ) {
  //     for ( auto& spimg : infill_label_vv.at(p) ) {
  // 	ev_infill_labelout[p]->Append( spimg );
  //     }
  //   }

  // }

  larcv::EventImage2D* ev_infill = (larcv::EventImage2D*)io.get_data(larcv::kProductImage2D,"infill");
  ev_infill->Emplace( std::move(infill_merged_out_v) );

  larcv::EventImage2D* ev_infill_label = (larcv::EventImage2D*)io.get_data(larcv::kProductImage2D,"inlabel");
  ev_infill_label->Emplace( std::move(in_label_v) );

  // cosmic mask r-cnn
  // ------------------
  larcv::EventClusterMask* ev_mask_v = (larcv::EventClusterMask*)io.get_data(larcv::kProductClusterMask,"mrcnn_masks");
  for ( size_t p=0; p<3; p++ ) {
    if ( p<ubmrcnn_result_vv.size() )
      ev_mask_v->emplace( std::move(ubmrcnn_result_vv.at(p)) );
  }
  
  // save entry: larcv
  LARCV_INFO() << "saving entry" << std::endl;
  io.save_entry();
  
  // save entry: larlite
  _out_larlite.set_id( e.id().run(), e.id().subRun(), e.id().event() );
  _out_larlite.next_event();

  // we clear entries ourselves
  LARCV_INFO() << "clearing entry" << std::endl;
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
int DLInterface::runSupera( art::Event& e, 
			    std::vector<larcv::Image2D>& wholeview_imgs,
			    larcv::EventChStatus** ev_chstatus ) {

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
    throw ::cet::exception("DLInterface") << "Could not locate recob::Wire data!" << std::endl;
  }
  LARCV_INFO() << "SET WIRE DATA POINTER TO SUPERA" << std::endl;
  _supera.SetDataPointer(*data_h,_wire_producer_name);

  // :chstatus
  auto supera_chstatus = _supera.SuperaChStatusPointer();
  if(supera_chstatus) {
    // Set database status
    LARCV_INFO() << "LOAD SUPERA CHSTATUS" << std::endl;
    auto const* geom = ::lar::providerFrom<geo::Geometry>();
    const lariov::ChannelStatusProvider& chanFilt = art::ServiceHandle<lariov::ChannelStatusService>()->GetProvider();
    for(size_t i=0; i < geom->Nchannels(); ++i) {
      auto const wid = geom->ChannelToWire(i).front();
      if (!chanFilt.IsPresent(i)) supera_chstatus->set_chstatus(wid.Plane, wid.Wire, ::larcv::chstatus::kNOTPRESENT);
      else supera_chstatus->set_chstatus(wid.Plane, wid.Wire, (short)(chanFilt.Status(i)));
    }
  }
  else {
    LARCV_INFO() << "SUPERA CHSTATUS NOT LOADED" << std::endl;
  }

  // execute supera
  bool autosave_entry = false;
  LARCV_INFO() << "PROCESS SUPERA EVENT: (" << e.id().run() << "," << e.id().subRun() << "," << e.id().event() << ")" << std::endl;
  _supera.process(e.id().run(),e.id().subRun(),e.id().event(), autosave_entry);

  // get the images
  LARCV_INFO() << "RETRIEVE ADC IMAGES: (" << e.id().run() << "," << e.id().subRun() << "," << e.id().event() << ")" << std::endl;
  auto ev_imgs  = (larcv::EventImage2D*) _supera.driver().io_mutable().get_data( larcv::kProductImage2D, "wire" );
  ev_imgs->Move( wholeview_imgs );

  // get chstatus
  if ( supera_chstatus ) {
    LARCV_INFO() << "RETRIEVE EVCHSTATUS FROM SUPERA" << std::endl;
    try {
      *ev_chstatus = (larcv::EventChStatus*) _supera.driver().io_mutable().get_data( larcv::kProductChStatus, "wire" );

      if ( _verbosity==larcv::msg::kDEBUG ) {
	LARCV_DEBUG() << "======================================" << std::endl;
	LARCV_DEBUG() << " CHECK CHSTATUS INFO" << std::endl;
	size_t nwires[3] = { 2400, 2400, 3456 };
	for ( size_t p=0; p<wholeview_imgs.size(); p++ ) {
	  LARCV_DEBUG() << " [plane " << p << "] --------- " << std::endl;
	  int nbad = 0;
	  auto& chstatus = (*ev_chstatus)->Status((larcv::PlaneID_t)p);
	  for (size_t w=0; w<nwires[p]; w++ ) {
	    if ( chstatus.Status(w)!=4 ) {
	      LARCV_DEBUG() << "  bad wire: " << w << std::endl;
	      nbad++;
	    }
	  }
	  LARCV_DEBUG() << " plane nbad wires: " << nbad << std::endl;
	}
	LARCV_DEBUG() << "======================================" << std::endl;
      }
    }
    catch (std::exception& e ) {
      std::stringstream errout;
      errout << "Trouble making Ch Status object.  Might try checking that status DB is OK" << std::endl;
      LARCV_DEBUG() << errout.str();
      throw cet::exception(errout.str());
    }
  }
  
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
  //std::cout << "DLInterface: superawire images made " << wholeview_v.size() << std::endl;
  // for (auto& img : wholeview_v ) {
  //   std::cout << img.meta().dump() << std::endl;
  // }  
  
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
 * produce subimage crops based on the location of the in-time flash
 *
 * passes the images into an instance of FixedCROIFromFlashAlgo (_croifromflashalgo member)
 *
 * @param[in] wholeview_v vector of whole-view images. one per plane.
 * @param[in] e art::Event from which we will get the opflash objects
 * @param[inout] splitimg_v vector of cropped images. many per plane.
 * @param[inout] splitroi_v vector ROIs for split images. many  per plane.
 *
 **/
int DLInterface::runCROIfromFlash( const std::vector<larcv::Image2D>& wholeview_v, 
				   art::Event& e,
				   std::vector<larcv::Image2D>& splitimg_v,
				   std::vector<larcv::ROI>&     splitroi_v ) {

  // first get the opflash objects from art
  art::Handle<std::vector<recob::OpFlash> > data_h;
  //  handle sub-name
  if(_opflash_producer_name.find(" ")<_opflash_producer_name.size()) {
    e.getByLabel(_opflash_producer_name.substr(0,_opflash_producer_name.find(" ")),
		 _opflash_producer_name.substr(_opflash_producer_name.find(" ")+1,_opflash_producer_name.size()-_opflash_producer_name.find(" ")-1),
		 data_h);
  }else{ e.getByLabel(_opflash_producer_name, data_h); }

  if(!data_h.isValid()) { 
    std::cerr<< "Attempted to load recob::Opflash data: " << _opflash_producer_name << std::endl;
    throw cet::exception("DLInterface") << "Could not locate recob::Opflash data!" << std::endl;
  }
  //std::cout << "Loaded opflash data" << std::endl;

  const float usec_min = 190*0.015625;
  const float usec_max = 320*0.015625;

  std::vector<larlite::opflash> intime_flashes_v;
  for ( auto const& artflash : *data_h ) {

    if ( artflash.Time()<usec_min || artflash.Time()>usec_max ) continue;

    // hmm, there is no getter for the number of entries in the PE vector. 
    // that seems wrong. we cheat about the number of PEs for now
    // next we have to make larlite objects
    std::vector<double> pe_v(32,0.0);
    for ( size_t ich=0; ich<32; ich++ ) {
      pe_v[ich] = artflash.PE(ich);
    }
    larlite::opflash llflash( artflash.Time(),    artflash.TimeWidth(),
			      artflash.AbsTime(), artflash.Frame(),
			      pe_v,
			      1, 1, 1,
			      artflash.YCenter(), artflash.YWidth(),
			      artflash.ZCenter(), artflash.ZWidth(),
			      artflash.WireCenters(),
			      artflash.WireWidths() );
    intime_flashes_v.emplace_back( std::move(llflash) );
  }
  //std::cout << "converted into larlite opflash" << std::endl;

  // pass them to the algo to get CROI    
  for ( auto& llflash : intime_flashes_v ) {
    std::vector<larcv::ROI> flashrois = _croifromflashalgo.findCROIfromFlash( llflash );
    for ( auto& roi : flashrois )
      splitroi_v.emplace_back( std::move(roi) );
  }
  //std::cout << "create roi from flash" << std::endl;

  // cropout the regions
  size_t nplanes = wholeview_v.size();
  size_t ncrops = 0;
  for (size_t plane=0; plane<nplanes; plane++ ) {
    const larcv::Image2D& wholeimg = wholeview_v[plane];
    for ( auto& roi : splitroi_v ) {
      const larcv::ImageMeta& bbox = roi.BB(plane);
      //std::cout << "crop from the whole image" << std::endl;
      try {
	larcv::Image2D crop = wholeimg.crop( bbox );
	splitimg_v.emplace_back( std::move(crop) );
      }
      catch( std::exception& e ) {
	throw cet::exception("DLInterface") << "error in cropping: " << e.what() << std::endl;
      }
      ncrops++;
    }
  }
  //std::cout << "cropped the regions: total " << ncrops << std::endl;

  // done
  return (int)ncrops;
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
  
  //std::cout << "Created " << msg_img_v.size() << " messages. Total image message size: " << img_msg_totsize << " (chars)" << std::endl;
  return img_msg_totsize;
}

/**
 * 
 * run ssnet via GPU server
 *
 * takes the list of split images then
 *  1. serializes them
 *  2. sends request to broker
 *  3. receives reply
 *  4. checks reply
 *  5. converts back to Image2D
 *
 * the client will send the request and wait (_client_timeout_secs) seconds
 * for the response to come in. The client will try to request
 * a max number of (_request_max_tries) times.  Upon receiving data,
 * the client will check to see that the data is the correct one, using
 * (run,subrun,event,iid) information. If not correct or incomplete, 
 * request is sent again, for a total of (_request_max_tries) tries.
 * 
 */
int DLInterface::runSSNetServer( const int run, const int subrun, const int event, 
				 const std::vector<larcv::Image2D>& wholeview_v, 
				 std::vector<larcv::Image2D>& splitimg_v, 
				 std::vector<larcv::ROI>& splitroi_v, 
				 std::vector<larcv::Image2D>& showerout_v,
				 std::vector<larcv::Image2D>& trackout_v ) {

  bool debug  = ( _verbosity==0 ) ? true : false;

  // mcc9 vs mcc8 scale factors
  float mcc9vs8_scale[3] = { 43.0/53.0, 43.0/52.0, 48.0/59.0 };
  float adc_threshold[3] = { 10.0, 10.0, 10.0 };

  // create the messages for the data (i.e. serialize)
  //std::cout << "RunSSNetServer: serialize" << std::endl;
  std::vector< zmq::message_t > zmsg_v[3];

  for ( size_t iimg=0; iimg<splitimg_v.size(); iimg++ ) {

    larcv::Image2D& img = splitimg_v.at(iimg);

    int plane = img.meta().plane();
    int iid   = zmsg_v[plane].size();

    // scale to mcc8 values and threshold
    img.scale_inplace( mcc9vs8_scale[plane] );
    std::vector<float>& imgdata = img.as_mod_vector();
    for ( size_t i=0; i<imgdata.size(); i++ ) 
      if ( imgdata[i]<adc_threshold[plane] )
	imgdata[i] = 0;

    // make a binary json string
    std::vector<std::uint8_t> bson 
      = larcv::json::as_bson( img, run, subrun, event, iid );
    zmq::message_t an_image( bson.size() );
    memcpy( an_image.data(), bson.data(), bson.size() );
    zmsg_v[ plane ].emplace_back( std::move(an_image) );

  }


  int n_timeout = 0; // number of times we've timedout waiting for reply
  int ntries    = 0; // number of tries to get complete response

  for ( size_t plane=0; plane<3; plane++ ) {

    // send the message according to majortomo protocol
    if ( debug ) {
      std::cout << "RunSSNetServer: process Plane[" << plane << "] images. "
		<< "nimages=" << zmsg_v[plane].size() 
		<< std::endl;
    }

    size_t nimgs = zmsg_v[plane].size();
    
    std::vector<int>             ok_v( nimgs, 0 ); // track good messages
    bool is_complete = false;                      // true, when all messages responses collected

    std::vector< larcv::Image2D > shr_v;           // shower images
    std::vector< larcv::Image2D > trk_v;           // track  images


    while ( !is_complete && ntries<_request_max_tries ) {

      ntries += 1;

      // send the images
      // we only sned images where ok_v[iimg]==False
      sendSSNetPlaneImages( (int)plane, ok_v, zmsg_v[plane], debug );

      // poll for a response
      bool isfinal = false; // final msg received
      std::vector< zmq::message_t > zreply_v; // reply container

      while ( !isfinal ) {

	zmq::poll( _poller[kSSNET], 1, _client_timeout_secs*100000 ); // last argument is in microseconds

	if ( n_timeout<_max_n_broker_reconnect
	     &&  _poller[kSSNET]->revents & ZMQ_POLLERR ) {
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
	else if ( _poller[kSSNET]->revents & ZMQ_POLLIN ) {

	  // receive multi-part: keep receiving until done
	  // expect reply with at least 4 parts: [empty] [WORKER HEADER] [PARTIAL/FINAL] [DATA ...]

	  // reset timeout counter, as we've successfully communicated with broker
	  n_timeout = 0;
	  
	  int more = 1;                         // more parts coming
	  std::vector< zmq::message_t > part_v; // store parts
	  int nparts = 0;

	  while ( more==1 && nparts<100 ) {
	    zmq::message_t reply;
	    _socket[kSSNET]->recv( &reply );
	    size_t more_size = sizeof(more);
	    _socket[kSSNET]->getsockopt(ZMQ_RCVMORE,&more,&more_size);
	    part_v.emplace_back( std::move(reply) );
	    if ( debug ) std::cout << "    received msg part=" << nparts << " more=" << more << std::endl;
	    nparts++;
	  }
	  if ( debug )  {
	    std::cout << "  Plane[" << plane << "]"
		      << "  Received in batch: " << part_v.size()  << ".  "
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
      }//end of isfinal loop


      if ( debug ) std::cout << " Number of replies: " << zreply_v.size() << std::endl;

      // we check if we got all of them back
      // we expect two images back per sent (track+shower scores)

      for ( auto& data : zreply_v ) {
	// convert into vector of uint8_t
	std::vector< std::uint8_t > bson( data.size(), 0 );
	memcpy( bson.data(), data.data(), data.size() );
	// convert into image
	nlohmann::json j = larcv::json::json_from_bson( bson );
	int rrun,rsubrun,revent,reid; // returned IDs
	larcv::json::rseid_from_json( j, rrun, rsubrun, revent, reid );
	larcv::Image2D img = larcv::json::image2d_from_json( j );
	if ( rrun==run && rsubrun==subrun && revent==event && reid>=0 && reid<(int)nimgs ) {
	  if ( ok_v[reid]<2 ) {
	    if ( debug ) std::cout << "  expected reply " << img.meta().dump() << " reid=" << reid << std::endl;
	    ok_v[ reid ]++;
	    if ( ok_v[reid]==2 )
	      trk_v.emplace_back( std::move(img) );
	    else if ( ok_v[reid]==1 )
	      shr_v.emplace_back( std::move(img) );
	  }
	  else {
	    if ( debug ) std::cout << "  repeat reply " << img.meta().dump() << " reid=" << reid << std::endl;
	  }
	}
	else {
	  std::cerr << "  Unexpected (run,subrun,event,id)" << std::endl;
	}
      }//end of zreply_v loop


      // check status of replies
      is_complete = true;
      for ( size_t ii=0; ii<nimgs; ii++ ) {
	if ( ok_v[ii]!=2 ) {
	  is_complete = false;
	  std::cerr << "  Missing reply imgid=" << ii << std::endl;
	}
      }
      
    }//end of request loop

    if ( is_complete ) {
      if ( debug ) std::cout << "  Replies complete!" << std::endl;
      for ( auto& shr : shr_v ) {
	showerout_v.emplace_back( std::move(shr) );
      }
      for ( auto& trk : trk_v ) {
	trackout_v.emplace_back( std::move(trk) );
      }
    }
    else {
      std::cerr << "Incomplete set of images processed for Plane=" << plane << std::endl;
      return 1;
    }
    
    // reset counters
    ntries = 0;
    n_timeout = 0;

  }//end of loop over planes
    
  // return ok status
  return 0;
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
				std::vector<larcv::Image2D>& showerout_v,
				std::vector<larcv::Image2D>& trackout_v) {

#ifdef HAS_TORCH

  size_t nimgs   = splitimg_v.size();


  //run the net!
  size_t iimg = 0;
  showerout_v.clear();
  showerout_v.reserve( nimgs+10 );
  trackout_v.clear();
  trackout_v.reserve( nimgs+10 );
  for ( auto& img : splitimg_v ) {
    //std::cout << "img[" << iimg << "] converting to aten::tensor" << std::endl;
    std::vector<torch::jit::IValue> input;
    input.push_back( larcv::torchutils::as_tensor( img ).reshape( {1,1,(int)img.meta().cols(),(int)img.meta().rows()} ) );
    size_t plane = img.meta().plane();
    at::Tensor output;
    try {
      output = _module_ubssnet.at(plane)->forward(input).toTensor();
      //std::cout << "img[" << iimg << ",plane=" << plane << "] network produced ssnet img dim=";
      // for ( int i=0; i<output.dim(); i++ )
      // 	std::cout << output.size(i) << " ";
      // std::cout << std::endl;
    }
    catch (std::exception& e) {
      throw cet::exception("DLInterface") << "module error while running img[" << iimg << "]: " << e.what() << std::endl;
    }

    // output is {1,3,H,W} with values being log(softmax)
    at::Tensor shower_slice = output.slice(1, 1, 2); // dim, start, end
    at::Tensor track_slice  = output.slice(1, 2, 3);
    //std::cout << "img[" << iimg << "] slice dim=";
    // for ( int i=0; i<shower_slice.dim(); i++ )
    //   std::cout << shower_slice.size(i) << " ";
    // std::cout << std::endl;


    // as img2d
    //std::cout << "img[" << iimg << "] converting out back to image2d" << std::endl;
    try {
      larcv::Image2D shrout = larcv::torchutils::image2d_fromtorch( shower_slice, 
								    splitimg_v[iimg].meta() );
      larcv::Image2D trkout = larcv::torchutils::image2d_fromtorch( track_slice,  
								    splitimg_v[iimg].meta() );
      //std::cout << "img[" << iimg << "] conversion complete. store." << std::endl;
      showerout_v.emplace_back( std::move(shrout) );
      trackout_v.emplace_back(  std::move(trkout) );
    }
    catch (std::exception& e ) {
      throw cet::exception("DLInterface") << "error in converting tensor->image2d: " << e.what() << std::endl;
    }
    
    iimg++;
  }
  
#endif // HAS_TORCH
  
  return 0;
}

/**
 * split infill image
 *
 */
void DLInterface::splitImageForInfill( const std::vector<larcv::Image2D>& adc_v, 
				       larcv::EventChStatus& ev_chstatus,
				       std::vector< larcv::Image2D >& label_v,
				       std::vector< std::vector<larcv::SparseImage> >& adc_crops_vv,
				       std::vector<std::vector<larcv::Image2D> >& label_crops_vv,
				       const float threshold,
				       const std::string infill_crop_cfg ) {

  // make images where dead channels are labeled
  // ---------------------------------------------
  
  //std::vector<larcv::Image2D> label_v;
  LARCV_INFO() << "Make label image for infill: ev_chstatus=" << &ev_chstatus << std::endl;

  label_crops_vv.clear();
  adc_crops_vv.clear();

  // blank label image
  label_v.clear();
  for ( auto const& img : adc_v ) {
    larcv::Image2D label(img.meta());
    label.paint(0.0);
    label_v.emplace_back( std::move(label) );
  }    
    
  LARCV_DEBUG() << ":: Make Chstatus Image" << std::endl;

  ublarcvapp::InfillDataCropper::ChStatusToLabels(label_v,&ev_chstatus);
    

  // Define PSet. Expect parameters above
  // -------------------------------------
  larcv::PSet cfg = larcv::CreatePSetFromFile( infill_crop_cfg, "UBSplitDetector" );

  // create whole-image splitter
  // -----------------------------
  LARCV_DEBUG() << ":: Configure Infill UBSplitDetector" << std::endl;
  ublarcvapp::UBSplitDetector ubsplit;
  ubsplit.configure(cfg);
  ubsplit.initialize();
  //if ( debug )
  //ubsplit.set_verbosity(larcv::msg::kDEBUG);

  ublarcvapp::UBSplitDetector ubsplit_label;
  ubsplit_label.configure(cfg);
  ubsplit_label.initialize();
  //if ( debug )
  //ubsplit_label.set_verbosity(larcv::msg::kDEBUG);
    
  // split image and dead channel label images
  // ------------------------------------------
  std::vector<larcv::Image2D> cropadc_v;
  std::vector<larcv::ROI>     adc_roi_v;
  LARCV_INFO() << ":: Run Infill splitter on ADC images" << std::endl;
  ubsplit.process( adc_v, cropadc_v, adc_roi_v );
    
  std::vector<larcv::Image2D> croplabel_v;
  std::vector<larcv::ROI> label_roi_v;
  LARCV_INFO() << ":: Run Infill splitter on ChStatus images" << std::endl;
  ubsplit_label.process( label_v, croplabel_v, label_roi_v );

  // convert crops into sparse images    
  //  --------------------------------
  size_t nsets = cropadc_v.size()/adc_v.size();
  std::vector<float> threshold_v(1,threshold);

  adc_crops_vv.clear();
  adc_crops_vv.resize(adc_v.size());
  label_crops_vv.clear();
  label_crops_vv.resize(adc_v.size());
  for ( size_t p=0; p<adc_v.size(); p++ ) {
    adc_crops_vv.at(p).reserve( nsets );
    label_crops_vv.at(p).reserve( nsets );
  }
  
  for ( size_t iset=0; iset<nsets; iset++ ) {
    for ( size_t p=0; p<adc_v.size(); p++ ) {
	
      int imgindex = iset*adc_v.size()+p;
      
      // get image2d
      auto& adcimg   = cropadc_v.at( imgindex );
      auto& labelimg = croplabel_v.at( imgindex );
	
      // convert into sparseimg
      larcv::SparseImage sparse( adcimg, labelimg, threshold_v );
      adc_crops_vv.at(p).emplace_back(std::move(sparse));
      label_crops_vv.at(p).emplace_back(std::move(labelimg));
    }
  }
    
}


/**
 * merge infill network output on crops into a wholeview image
 *
 */
void DLInterface::stitchSparseCrops( const std::vector< std::vector<larcv::SparseImage> >& netout_vv,
				     const std::vector<larcv::Image2D>& wholeview_adc_v,
				     larcv::EventChStatus& ev_chstatus,
				     std::vector< larcv::Image2D >& mergedout_v ) {

  // create final output image
  mergedout_v.clear();
  
  // and image to track number of times pixel is filled
  std::vector<larcv::Image2D> overlap_v;
  
  // create images
  for ( size_t plane=0; plane<wholeview_adc_v.size(); plane++ ) {
    larcv::Image2D img( wholeview_adc_v.at(plane).meta() );
    img.paint(0.0);
    mergedout_v.emplace_back( std::move(img) );
    
    larcv::Image2D overlap(wholeview_adc_v.at(plane).meta());
    overlap.paint(0);
    overlap_v.emplace_back( std::move(overlap) );
  }


  // accumulate inputs
  for ( size_t plane=0; plane<wholeview_adc_v.size(); plane++ ) {
    
    larcv::ImageMeta merge_meta = mergedout_v.at(plane).meta();
    
    ublarcvapp::InfillImageStitcher stitcher;

    for ( auto const& netout : netout_vv.at(plane) ) {
      
      // should be 1 in this vector: the predicted in-fill ADC values
      std::vector<larcv::Image2D> outdense = netout.as_Image2D();
      LARCV_DEBUG() << ":: " << outdense.at(0).meta().dump() << std::endl;
      
      stitcher.Croploop( merge_meta,
			 outdense.at(0), 
			 mergedout_v.at(plane),
			 overlap_v.at(plane) );
    }
    
    // finish merged image
    stitcher.Overlayloop(plane,
			 merge_meta,
			 mergedout_v.at(plane),
			 overlap_v.at(plane), 
			 wholeview_adc_v, 
			 ev_chstatus);
    
  }//end of plane loop
    
}


/**
 * 
 * merge ssnet output into a wholeview image
 *
 * 
 */
void DLInterface::mergeSSNetOutput( const std::vector<larcv::Image2D>& wholeview_v, 
				    const std::vector<larcv::ROI>& splitroi_v,
				    const std::vector<larcv::Image2D>& showerout_v,
				    const std::vector<larcv::Image2D>& trackout_v,
				    std::vector<larcv::Image2D>& showermerged_v,
				    std::vector<larcv::Image2D>& trackmerged_v) {
				    
  
  // make output images
  showermerged_v.clear();
  trackmerged_v.clear();
  for ( auto const& img : wholeview_v ) {
    larcv::Image2D shrout( img.meta() );
    shrout.paint(0);
    showermerged_v.emplace_back( std::move(shrout) );

    larcv::Image2D trkout( img.meta() );
    trkout.paint(0);
    trackmerged_v.emplace_back( std::move(trkout) );
  }

  // loop through ssnetoutput, put values into image
  for ( size_t iimg=0; iimg<showerout_v.size(); iimg++ ) {
    auto const& shrout = showerout_v[iimg];
    auto& shrmerge     = showermerged_v.at( shrout.meta().plane() );
    shrmerge.overlay( shrout, larcv::Image2D::kOverWrite );
  }

  for ( size_t iimg=0; iimg<trackout_v.size(); iimg++ ) {
    auto const& trkout = trackout_v[iimg];
    auto& trkmerge     = trackmerged_v.at( trkout.meta().plane() );
    trkmerge.overlay( trkout, larcv::Image2D::kOverWrite );
  }
}

/**
 * create ubdldata::pixeldata objects for art::Event
 *
 *
 */
void DLInterface::saveSSNetArtProducts( art::Event& ev, 
					const std::vector<larcv::Image2D>& wholeimg_v,
					const std::vector<larcv::Image2D>& showermerged_v,
					const std::vector<larcv::Image2D>& trackmerged_v ) {
  
  std::unique_ptr< std::vector<ubdldata::pixeldata> > ppixdata_v(new std::vector<ubdldata::pixeldata>);
  LARCV_INFO() << "saving ssnet products into art event" << std::endl;

  if ( showermerged_v.size()==0 || trackmerged_v.size()==0 ) {
    // fill empty
    ev.put( std::move(ppixdata_v), "ssnet" );
    return;
  }
    
  for ( auto const& adc : wholeimg_v ) {
    int planeid           = (int)adc.meta().plane();
    float pix_threshold   = _pixelthresholds_forsavedscores.at(planeid);
    auto const& showerimg = showermerged_v.at(planeid);
    auto const& trackimg  = trackmerged_v.at(planeid);

    std::vector< std::vector<float> > pixdata_v;
    // we reserve enough space to fill the whole image, but we shouldn't use all of the space
    pixdata_v.reserve( adc.meta().rows()*adc.meta().cols() );

    size_t npixels = 0;
    for ( size_t r=0; r<adc.meta().rows(); r++ ) {
      float tick = adc.meta().pos_y(r);
      for ( size_t c=0; c<adc.meta().cols(); c++ ) {

	// each pixel
	
	if ( adc.pixel(r,c)<pix_threshold ) continue;

	float wire=adc.meta().pos_x(c);

	std::vector<float> pixdata = { (float)wire, (float)tick, 
				       (float)showerimg.pixel(r,c), (float)trackimg.pixel(r,c) };
	
	pixdata_v.push_back( pixdata );

	npixels++;
      }
    }
  
    ubdldata::pixeldata out( pixdata_v,
			     adc.meta().min_x(), adc.meta().min_y(), 
			     adc.meta().max_x(), adc.meta().max_y(),
			     (int)adc.meta().cols(), (int)adc.meta().rows(),
			     (int)adc.meta().plane(), 4, 0 );

    ppixdata_v->emplace_back( std::move(out) );
  }
  
  ev.put( std::move(ppixdata_v), "ssnet" );
}

/**
 * create ubdldata::pixeldata objects for art::Event
 *
 *
 */
void DLInterface::saveLArFlowArtProducts( art::Event& ev, 
					  const std::vector<larcv::Image2D>& wholeimg_v,
					  const std::vector<larcv::SparseImage>& larflow_results_v,
					  std::vector<larlite::larflow3dhit>& larflow_hit_v,
					  const std::vector<larcv::Image2D>& showermerged_v,
					  const std::vector<larcv::Image2D>& trackmerged_v ) {

  std::unique_ptr< std::vector<ubdldata::pixeldata> > larflowpix_v(new std::vector<ubdldata::pixeldata>);
  std::unique_ptr< std::vector<ubdldata::pixeldata> > larflowhit_v(new std::vector<ubdldata::pixeldata>);
  LARCV_INFO() << "saving LARFLOW products into art event" << std::endl;

  if ( larflow_results_v.size()==0 ) {
    // fill empty
    ev.put( std::move(larflowpix_v), "larflow" );
    ev.put( std::move(larflowhit_v), "larflowhits" );
    return;
  }

  // store sparse image data
  for ( auto const& spimg : larflow_results_v ) {
    const larcv::ImageMeta& meta = spimg.meta(0);
    int planeid                  = (int)meta.plane();
    int nfeatures                = spimg.nfeatures();

    std::vector< std::vector<float> > pixdata_v;
    size_t npts = spimg.pixellist().size()/( spimg.nfeatures()+2 );
    pixdata_v.reserve( npts );

    for ( size_t ipt=0; ipt<npts; ipt++ ) {
      int idx = ipt*(nfeatures + 2);
      float tick = meta.pos_y( spimg.pixellist()[idx]  );
      float wire = meta.pos_x( spimg.pixellist()[idx+1] );
      float flow = spimg.pixellist()[idx+2];

      std::vector<float> pixdata = { (float)wire, (float)tick, (float)flow };
      pixdata_v.push_back( pixdata );
    }
  
    ubdldata::pixeldata out( pixdata_v,
			     meta.min_x(), meta.min_y(), 
			     meta.max_x(), meta.max_y(),
			     meta.cols(),  meta.rows(),
			     (int)planeid, 3, 1 );
    larflowpix_v->emplace_back( std::move(out) );

  }
  ev.put( std::move(larflowpix_v), "larflow" );

  bool has_ssnet = (showermerged_v.size()>0) ? true : false;
  const larcv::ImageMeta& metay = wholeimg_v.at(2).meta();

  std::vector< std::vector<float> > hitdata_v;
  hitdata_v.reserve(larflow_hit_v.size());

  for ( size_t ihit=0; ihit<larflow_hit_v.size(); ihit++ ) {
    auto& hit = larflow_hit_v[ihit];
    
    // incorporate ssnet
    hit.shower_score = 0;
    hit.track_score  = 0;
    if ( has_ssnet ) {
      if ( hit.tick>metay.min_y() && hit.tick<metay.max_y() 
	   && hit.srcwire>=metay.min_x() && hit.srcwire<metay.max_x() ) {
	hit.shower_score = showermerged_v[2].pixel( metay.row(hit.tick), metay.col(hit.srcwire) );
	hit.track_score  = trackmerged_v[2].pixel(  metay.row(hit.tick), metay.col(hit.srcwire) );
      }
    }
    
    std::vector<float> hitdata = { (float)hit.srcwire, (float)hit.tick,
				   hit[0], hit[1], hit[2], hit.shower_score, hit.track_score };
    hitdata_v.emplace_back( std::move(hitdata) );
  }
  
  ubdldata::pixeldata hitpixout( hitdata_v,
				 metay.min_x(), metay.min_y(), 
				 metay.max_x(), metay.max_y(),
				 metay.cols(),  metay.rows(),
				 (int)2, 7, 1 );
  larflowhit_v->emplace_back( std::move(hitpixout) );
  ev.put( std::move(larflowhit_v), "larflowhits" );
}

/**
 * create ubdldata::pixeldata objects for art::Event for Infill
 *
 * @param[inout] ev art Event container we will add results to
 * @param[in] infill_stitched_v wholeview-image containing result of infill inlaid into dead regions
 * @param[in] infill_label_v wholeview-image marking pixels on dead region
 */
void DLInterface::saveInfillArtProducts( art::Event& ev, 
					 const std::vector<larcv::Image2D>& infill_stitched_v,
					 const std::vector<larcv::Image2D>& infill_label_v ) {
  
  std::unique_ptr< std::vector<ubdldata::pixeldata> > infillpix_v(new std::vector<ubdldata::pixeldata>);

  LARCV_INFO() << "saving INFILL products into art event" << std::endl;

  if ( infill_stitched_v.size()==0 ) {
    // fill empty
    ev.put( std::move(infillpix_v), "infill" );
    return;
  }
  
  // make sparse image and save it
  std::vector<float> infill_thresholds_v = { 10.0, 0.5 };
  std::vector<int>   require_pixel = { 1, 1 };

  for ( size_t p=0; p<infill_stitched_v.size(); p++ ) {
    const larcv::Image2D& stitch = infill_stitched_v.at(p);
    const larcv::Image2D& label  = infill_label_v.at(p);
    const larcv::ImageMeta& meta = stitch.meta();
    int nfeatures                = 2;
    int planeid                  = meta.plane();
    
    std::vector<const larcv::Image2D*> pimg_v = { &stitch, &label };
    larcv::SparseImage spimg( pimg_v, infill_thresholds_v, require_pixel );
    
    size_t npts = spimg.pixellist().size()/(2+nfeatures);
    
    std::vector< std::vector<float> > pixdata_v;
    for ( size_t ipt=0; ipt<npts; ipt++ ) {
      int idx = ipt*(nfeatures + 2);
      float tick = meta.pos_y( spimg.pixellist()[idx]  );
      float wire = meta.pos_x( spimg.pixellist()[idx+1] );
      float fill  = spimg.pixellist()[idx+2];
      float label = spimg.pixellist()[idx+3];

      std::vector<float> pixdata = { (float)wire, tick, fill, label };
      pixdata_v.push_back( pixdata );
    }
    
    ubdldata::pixeldata out( pixdata_v,
			     meta.min_x(), meta.min_y(), 
			     meta.max_x(), meta.max_y(),
			     meta.cols(),  meta.rows(),
			     (int)planeid, 4 );
    infillpix_v->emplace_back( std::move(out) );
    
  }
  ev.put( std::move(infillpix_v), "infill" );
  
}

/**
 * create ubdldata::pixeldata objects for art::Event for Cosmic Mask-RCNN.
 *
 * @param[inout] ev art Event container we will add results to
 * @param[in] wholeview_adc_v wholeview-image of ADC values
 * @param[in] ubmrcnn_result_vv for each plane, a vector containing clustermask from network
 */
void DLInterface::saveCosmicMRCNNArtProducts( art::Event& ev, 
					      const std::vector< larcv::Image2D >& wholeview_adc_v,
					      const std::vector< std::vector<larcv::ClusterMask> >& ubmrcnn_result_vv ) {
  
  std::unique_ptr< std::vector<ubdldata::pixeldata> > ubmrcnn_pix_v(new std::vector<ubdldata::pixeldata>);

  LARCV_INFO() << "saving Cosmic MRCNN products into art event" << std::endl;

  if ( wholeview_adc_v.size()==0 ) {
    // fill empty
    ev.put( std::move(ubmrcnn_pix_v), "ubmrcnn" );
    return;
  }
  
  // store result cluster masks into ubmrcnn_pix_v
  LARCV_INFO() << "size of results vector: " << ubmrcnn_result_vv.size() << std::endl;
  for ( size_t p=0; p<ubmrcnn_result_vv.size(); p++ ) {

    const larcv::Image2D& adc      = wholeview_adc_v.at(p);
    const larcv::ImageMeta& meta   = adc.meta();
    const std::vector<larcv::ClusterMask>& mask_v = ubmrcnn_result_vv[p];

    int planeid = meta.plane();
    
    LARCV_DEBUG() << "number of clustermasks in plane[" << planeid << "]: " << mask_v.size() << std::endl;
    
    int imask = 0;
    for ( auto const& mask : mask_v ) {

      const std::vector<float> mask_pts = mask.as_vector_mask_no_convert();
      size_t npts = mask_pts.size()/2;
      LARCV_DEBUG() << "  plane[" << planeid << "] mask[" << imask << "]: npts= " << npts << std::endl;
    
      std::vector< std::vector<float> > pixdata_v;
      for ( size_t ipt=0; ipt<npts; ipt++ ) {
	int idx = ipt*2;

	try {
	  std::vector<float> pixdata = { mask_pts[idx], mask_pts[idx+1] };
	  pixdata_v.push_back( pixdata );
	}
	catch( std::exception& e ) {
	  LARCV_WARNING() << "  mask pt[" << ipt << "] error" << std::endl;
	}
      }//end of point loop
    
      std::vector<float> bbox = mask.as_vector_box_no_convert();
      LARCV_DEBUG() << "  mask bbox (size=" << bbox.size() << "): " 
		    << " xmin=" << bbox[0] 
		    << " xmax=" << bbox[2]
		    << " ymin=" << bbox[1]
		    << " ymax=" << bbox[3]
		    << std::endl;

      // for last point, we store (interaction class, probability_of_class)
      std::vector<float> metadata = { (float)mask.type, mask.probability_of_class };
      LARCV_DEBUG() << "  add metadata entry: classtype=" << metadata[0] << "  classprob=" << metadata[1] << std::endl;
      pixdata_v.push_back( metadata );
      LARCV_DEBUG() << "  pixdata_v.size()=" << pixdata_v.size() << std::endl;

      try {
	ubdldata::pixeldata out( pixdata_v,
				 bbox[0], bbox[1], bbox[2], bbox[3],
				 (int)(bbox[2]-bbox[0]),  
				 (int)(bbox[3]-bbox[1]),
				 (int)planeid, 2, (int)mask.type );

	ubmrcnn_pix_v->emplace_back( std::move(out) );

	LARCV_DEBUG() << "  transer to output container[ p=" << (int)planeid << " ] " << std::endl;
      }
      catch (std::exception& e ) {
	LARCV_CRITICAL() << "failed to create pixeldata object: " << e.what() << std::endl;
	throw cet::exception("DLInterface") << " filed to create pixeldata object @ " << __FUNCTION__ << ".L" << __LINE__ << std::endl;
      }
      
    }// end of mask loop
  }//end of plane loop
  ev.put( std::move(ubmrcnn_pix_v), "ubmrcnn" );
  
}

/**
 * 
 * run sparse larflow via GPU server
 *
 * takes the list of split images then
 *  1. serializes them
 *  2. sends request to broker
 *  3. receives reply
 *  4. checks reply
 *  5. converts back to Image2D
 *
 * the client will send the request and wait (_client_timeout_secs) seconds
 * for the response to come in. The client will try to request
 * a max number of (_request_max_tries) times.  Upon receiving data,
 * the client will check to see that the data is the correct one, using
 * (run,subrun,event,iid) information. If not correct or incomplete, 
 * request is sent again, for a total of (_request_max_tries) tries.
 * 
 */
int DLInterface::runLArFlowServer( const int run, const int subrun, const int event, 
				   const std::vector<larcv::Image2D>& wholeview_v, 
				   std::vector<larcv::Image2D>& splitimg_v, 
				   std::vector<larcv::ROI>& splitroi_v, 
				   std::vector<larcv::SparseImage>& larflow_results_v,
				   std::vector<larlite::larflow3dhit>& larflow_hit_v ) {
  
  bool debug  = ( _verbosity==0 ) ? true : false;
  _larflow_server->processCroppedLArFlowViaServer( splitimg_v, 
						   run, subrun, event, 
						   10.0, 
						   larflow_results_v, 
						   debug );

  larflow_hit_v = _larflow_server->croppedFlow2hits( larflow_results_v,
						     wholeview_v,
						     _ubcrop_trueflow_cfg,
						     10.0,
						     debug );

  // send the message according to majortomo protocol
  LARCV_INFO() << "RunLArFlowServer: worker returned " 
	       << larflow_results_v.size() << " images and " 
	       << larflow_hit_v.size() << " hits"
	       << std::endl;
  
  // return ok status
  return 0;
}

/**
 * 
 * run sparse infill via GPU server through the Infill interface.
 * 
 * @param[in] run Run number.
 * @param[in] subrun Subrun number.
 * @param[in] event Event number.
 * @param[out] adc_crops_vv For each plane, the adc cropped images used in sparse form.
 * @param[out] results_vv   For each plane, the net output used in sparse form.
 * @param[in] threshold ADC pixel threshold used by infill machinery.
 */
int DLInterface::runInfillServer( const int run, const int subrun, const int event, 
				  std::vector<std::vector<larcv::SparseImage> >& adc_crops_vv,
				  std::vector<std::vector<larcv::SparseImage> >& results_vv,
				  const float threshold ) {
  
  bool debug  = ( _verbosity==0 ) ? true : false;
  LARCV_INFO() << "Run Infill Server" << std::endl;

  _infill_server->processSparseCroppedInfillViaServer( adc_crops_vv,
						       run, subrun, event, 
						       results_vv, 
						       threshold,
						       debug );

  LARCV_INFO() << "Finished Infill Server" << std::endl;
  // return ok status
  return 0;
}
/**
 * 
 * run sparse infill via GPU server through the Infill interface.
 * 
 * @param[in] run Run number.
 * @param[in] subrun Subrun number.
 * @param[in] event Event number.
 * @param[out] adc_crops_vv For each plane, the adc cropped images used in sparse form.
 * @param[out] results_vv   For each plane, the net output used in sparse form.
 * @param[in] threshold ADC pixel threshold used by infill machinery.
 */
int DLInterface::runInfill_cpu( const int run, const int subrun, const int event, 
				std::vector<std::vector<larcv::SparseImage> >& adc_crops_vv,
				std::vector<std::vector<larcv::SparseImage> >& results_vv ) {
  
  bool debug  = ( _verbosity==0 ) ? true : false;

  _sparseinfill_script->run_sparse_cropped_infill( adc_crops_vv,
						   run, subrun, event,
						   results_vv,
						   debug );

  
  return 0;
}
  

/**
 * 
 * run ubmrcnn via server through the CosmicMRCNN interface.
 * 
 * @param[in] run Run number.
 * @param[in] subrun Subrun number.
 * @param[in] event Event number.
 * @param[in] wholeview_v Vector of wholeview ADC images.
 * @param[out] results_vv   For each plane, the net output used in sparse form.
 * @return interger giving status. [0] is good.
 *
 */
int DLInterface::runCosmicMRCNNServer( const int run, const int subrun, const int event, 
				       const std::vector<larcv::Image2D>& wholeview_v, 
				       std::vector<std::vector<larcv::ClusterMask> >& results_vv ) {
  
  bool debug  = ( _verbosity==0 ) ? true : false;
  LARCV_INFO() << "Run UBCosmicMRCNN Server" << std::endl;

  _ubmrcnn_server->processViaServer( wholeview_v,
				     run, subrun, event, 
				     results_vv,
				     debug );
  
  LARCV_INFO() << "Run UBCosmicMRCNN Server" << std::endl;
  // return ok status
  return 0;
}

/**
 * 
 * run ubmrcnn via server through the CosmicMRCNN interface.
 * 
 * @param[in] run Run number.
 * @param[in] subrun Subrun number.
 * @param[in] event Event number.
 * @param[in] wholeview_v Vector of wholeview ADC images.
 * @param[out] results_vv   For each plane, the net output used in sparse form.
 * @return interger giving status. [0] is good.
 *
 */
int DLInterface::runCosmicMRCNN_cpu( const int run, const int subrun, const int event, 
				     const std::vector<larcv::Image2D>& wholeview_v, 
				     std::vector<std::vector<larcv::ClusterMask> >& results_vv ) {

  std::vector< std::vector< larcv::ClusterMask> > masks_vv;
  _ubmrcnn_script->run_cosmic_mrcnn( wholeview_v,
				     run, subrun, event,
				     10.0, masks_vv );

  for ( size_t p=0; p<3; p++ ) {
    for ( auto& mask : masks_vv.at(p) ) {
      results_vv.at(p).emplace_back( mask );
    }
  }
  
  return 0;
}

/**
 * 
 * initialize variables before job is run.
 *
 * 
 */
void DLInterface::beginJob()
{
  _supera.initialize();
  _out_larlite.open();
  _larflow_server = nullptr;
  _infill_server  = nullptr;
  _ubmrcnn_server = nullptr;

  _context = nullptr;

  for ( size_t i=0; i<kNUM_NETS; i++ ) {
    _socket[i] = nullptr;
    _poller[i] = nullptr;
  }

  if (_connect_to_server ) {
    openServerSocket();
  }
  
  if ( _ssnet_mode==kPyTorchCPU )
    loadNetwork_PyTorchCPU();


  // python script interface for ubmrcnn
  _ubmrcnn_script = nullptr;
  if ( _cosmic_mrcnn_mode==kPyTorchCPU ) {
    _ubmrcnn_script = new ubcv::ubdlintegration::PyNetCosmicMRCNN( _ubmrcnn_weight_file_v, _ubmrcnn_config_file_v );
  }

  // python script interface for infill
  _sparseinfill_script = nullptr;
  if ( _infill_mode==kPyTorchCPU ) {
    _sparseinfill_script = new ubcv::ubdlintegration::PyNetSparseInfill(  _sparseinfill_weight_file_v );
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
  _out_larlite.close();
  std::cout << "DLInterface::endJob -- finished" << std::endl;

  if ( _ubmrcnn_script )
    delete _ubmrcnn_script;

  if ( _sparseinfill_script )
    delete _sparseinfill_script;
    
}

/**
 * 
 * load pytorch network from torch script file
 *
 * 
 */
void DLInterface::loadNetwork_PyTorchCPU() {
#ifdef HAS_TORCH
  _module_ubssnet.clear();
  for ( size_t iscript=0; iscript<_pytorch_net_script.size(); iscript++ ) {
    std::cout << "Loading network[" << iscript << "] from " 
	      << _pytorch_net_script[iscript] << " .... " << std::endl;
    
    std::string scriptpath;
    cet::search_path finder("FHICL_FILE_PATH");
    if( !finder.find_file(_pytorch_net_script[iscript], scriptpath) )
      throw cet::exception("DLInterface") << "Unable to find torch script in "  << finder.to_string() << "\n";
    std::cout << "LOADING pytorch model data: " << scriptpath << std::endl;

    _module_ubssnet.push_back( torch::jit::load( scriptpath ) );

  }
  std::cout << "Networks Loaded" << std::endl;
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

  std::string broker = _broker_address+":"+_broker_port;
  LARCV_INFO() << "Connecting to Broker @ " << broker << std::endl;

  // setup server interfaces
  if ( _ssnet_mode==kServer ) {
    _socket[kSSNET]  = new zmq::socket_t( *_context, ZMQ_DEALER );
    _socket[kSSNET]->connect( broker );  
    _poller[kSSNET] = new zmq::pollitem_t{ *_socket[kSSNET], 0, ZMQ_POLLIN, 0 };
  }
  else {
    _socket[kSSNET] = nullptr;
    _poller[kSSNET] = nullptr;
  }

  if ( _larflow_mode==kServer ) {
    _socket[kLARFLOW]  = new zmq::socket_t( *_context, ZMQ_DEALER );
    _socket[kLARFLOW]->connect( broker );  
    _poller[kLARFLOW] = new zmq::pollitem_t{ *_socket[kLARFLOW], 0, ZMQ_POLLIN, 0 };    
    _larflow_server = new dl::LArFlow( _socket[kLARFLOW], _poller[kLARFLOW], dl::LArFlow::kDENSE );
  }
  else {
    _socket[kLARFLOW] = nullptr;
    _poller[kLARFLOW] = nullptr;
    _larflow_server = nullptr;
  }

  if ( _infill_mode==kServer ) {
    _socket[kINFILL]  = new zmq::socket_t( *_context, ZMQ_DEALER );
    _socket[kINFILL]->connect( broker );  
    _poller[kINFILL] = new zmq::pollitem_t{ *_socket[kINFILL], 0, ZMQ_POLLIN, 0 };
    _infill_server   = new dl::Infill( _socket[kINFILL], _poller[kINFILL] );
  }
  else {
    _socket[kINFILL] = nullptr;
    _poller[kINFILL] = nullptr;
    _infill_server = nullptr;
  }

  if ( _cosmic_mrcnn_mode==kServer ) {
    _socket[kUBMRCNN]  = new zmq::socket_t( *_context, ZMQ_DEALER );
    _socket[kUBMRCNN]->connect( broker );  
    _poller[kUBMRCNN] = new zmq::pollitem_t{ *_socket[kUBMRCNN], 0, ZMQ_POLLIN, 0 };
    _ubmrcnn_server   = new dl::CosmicMRCNN( _socket[kUBMRCNN], _poller[kUBMRCNN] );
  }
  else {
    _socket[kUBMRCNN] = nullptr;
    _poller[kUBMRCNN] = nullptr;
    _ubmrcnn_server = nullptr;
  }

}

/**
 * 
 * close the server socket
 *
 * 
 */
void DLInterface::closeServerSocket() {

  std::cout << "DLInterface: close server socket" << std::endl;

  // destroy the interfaces
  if (_larflow_server )
    delete _larflow_server;

  if (_infill_server)
    delete _infill_server;

  if (_ubmrcnn_server)
    delete _ubmrcnn_server;

  // close zmq socket
  for ( size_t i=0; i<kNUM_NETS; i++ ) {

    if ( _socket[i] )
      delete _socket[i];
    if ( _poller[i] )
      delete _poller[i];

    _socket[i] = nullptr;
    _poller[i] = nullptr;
  }

  if ( _context )
    delete _context;

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
    _socket[kSSNET]->send (request);
    
    //  Get the reply.
    zmq::message_t reply;
    _socket[kSSNET]->recv (&reply);
    std::cout << "Received World " << request_nbr << std::endl;
  }
  
  std::cout << "end of sendDummyMessage()" << std::endl;
}

size_t DLInterface::sendSSNetPlaneImages( const int plane,
					  const std::vector<int>& reply_ok,
					  std::vector< zmq::message_t >& zmsg_v,
					  bool debug ) {
  
  // service name
  char service_name[20];
  sprintf(service_name,"ubssnet_plane%d",plane);

  // message header codes
  const std::uint8_t empty = 0;
  const char request       = '\002';
  zmq::message_t header_msg (6);
  memcpy (header_msg.data (), "MDPC02", 6);


  // header
  // ------
  
  if (debug) std::cout << "  debug: send empty" << std::endl;
  _socket[kSSNET]->send( (char*)empty,       0, ZMQ_SNDMORE );
  if (debug) std::cout << "  debug: send header" << std::endl;
  _socket[kSSNET]->send( header_msg.data(),  6, ZMQ_SNDMORE );
  if (debug) std::cout << "  debug: send request command" << std::endl;
  _socket[kSSNET]->send( (char*)&request, 1, ZMQ_SNDMORE );
  if (debug) std::cout << "  debug: send service name" << std::endl;
  _socket[kSSNET]->send( service_name,   14, ZMQ_SNDMORE );

  size_t nimgs = zmsg_v.size();
  
  // last image
  size_t last_index = 0;
  for ( size_t imsg=0; imsg<nimgs; imsg++ ) {
    if ( reply_ok[imsg] ) continue;
    last_index = imsg;
  }

  // data
  // ----
  size_t nsent = 0;
  for ( size_t imsg=0; imsg<nimgs; imsg++ ) {
    if ( reply_ok[imsg]>0 ) continue;
    zmq::message_t& zmsg = zmsg_v.at(imsg);

    if (imsg!=last_index) {
      if ( debug ) std::cout << "  debug: send img[" << imsg << "]" << std::endl;
      _socket[kSSNET]->send( zmsg.data(), zmsg.size(), ZMQ_SNDMORE );
      nsent++;
    }
    else {
      if ( debug ) std::cout << "  debug: send final img[" << imsg << "]" << std::endl;
      _socket[kSSNET]->send( zmsg.data(), zmsg.size(), 0 );
      nsent++;
    }
  }
  
  return nsent;
}


DEFINE_ART_MODULE(DLInterface)
