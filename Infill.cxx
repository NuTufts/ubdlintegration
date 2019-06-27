#include "Infill.h"

#include "larcv/core/Base/LArCVBaseUtilFunc.h"
#include "larcv/core/Base/PSet.h"
#include "larcv/core/DataFormat/EventROI.h"
#include "larcv/core/DataFormat/ROI.h"

#include "ublarcvapp/UBImageMod/InfillImageStitcher.h"
#include "ublarcvapp/UBImageMod/InfillDataCropper.h"
#include "ublarcvapp/UBImageMod/UBSplitDetector.h"

namespace dl {

  /**
   * process whole adc images through infill network
   *
   * @param[in] adc_v whole adc images
   * @param[in] run run number
   * @param[in] subrun subrun number
   * @param[in] event event number
   * @param[inout] results_vv contains vector of images for each plane
   * @param[in] debug if true, run server functions in debug mode.
   */
  void Infill::processWholeImageInfillViaServer( const std::vector<larcv::Image2D>& adc_v,
						 larcv::EventChStatus& ev_chstatus,
						 const int run, const int subrun, const int event, 
						 std::vector<std::vector<larcv::SparseImage> >& adc_crops_vv,
						 std::vector<std::vector<larcv::SparseImage> >& results_vv,
						 std::vector<larcv::Image2D>& stitched_output_v,
						 const float threshold,
						 const std::string infill_crop_cfg,
						 bool debug ) {

    // make images where dead channels are labeled
    // ---------------------------------------------
    std::vector<larcv::Image2D> label_v;
    for ( auto const& img : adc_v ) {
      larcv::Image2D label(img.meta());
      label.paint(0.0);
      label_v.emplace_back( img );
    }    
    ublarcvapp::InfillDataCropper::ChStatusToLabels(label_v,&ev_chstatus);

    // std::string scfg="""Verbosity: 3
    //     InputProducer: \"wire\"
    //     OutputBBox2DProducer: \"detsplit\"
    //     CropInModule: true
    //     OutputCroppedProducer: \"detsplit\"
    //     BBoxPixelHeight: 512
    //     BBoxPixelWidth: 496
    //     CoveredZWidth: 310
    //     FillCroppedYImageCompletely: true
    //     DebugImage: false
    //     MaxImages: -1
    //     MaxRandomAttempts: 4
    //     MinFracPixelsInCrop: -0.0001
    //     TickForward: false
    //     """
    // Define PSet. Expect parameters above
    // -------------------------------------
    larcv::PSet cfg = larcv::CreatePSetFromFile( infill_crop_cfg, "UBSplitDetector" );

    // create whole-image splitter
    // -----------------------------
    ublarcvapp::UBSplitDetector ubsplit;
    ubsplit.configure(cfg);
    ubsplit.initialize();
    if ( debug )
      ubsplit.set_verbosity(larcv::msg::kDEBUG);
    
    // split image and dead channel label images
    // ------------------------------------------
    std::vector<larcv::Image2D> cropadc_v;
    std::vector<larcv::ROI> adc_roi_v;
    ubsplit.process( adc_v, cropadc_v, adc_roi_v );
    
    std::vector<larcv::Image2D> croplabel_v;
    std::vector<larcv::ROI> label_roi_v;
    ubsplit.process( label_v, croplabel_v, label_roi_v );

    // convert crops into sparse images    
    //  --------------------------------
    size_t nsets = cropadc_v.size()/adc_v.size();
    std::vector<float> threshold_v(1,threshold);

    adc_crops_vv.clear();
    adc_crops_vv.resize(adc_v.size());
    for ( size_t p=0; p<adc_v.size(); p++ )
      adc_crops_vv.at(p).reserve( nsets );
    
    for ( size_t iset=0; iset<nsets; iset++ ) {
      for ( size_t p=0; p<adc_v.size(); p++ ) {
	
	int imgindex = iset*adc_v.size()+p;

	// get image2d
	auto& adcimg   = cropadc_v.at( imgindex );
	auto& labelimg = croplabel_v.at( imgindex );
	
	// convert into sparseimg
	larcv::SparseImage sparse( adcimg, labelimg, threshold_v );
	adc_crops_vv.at(p).emplace_back(std::move(sparse));
      }
    }

    
    // process sparse crops
    processSparseCroppedInfillViaServer( adc_crops_vv, 
					 run, subrun, event,
					 results_vv,
					 threshold, 
					 debug );
    
    // stitch infill crops
    stitchSparseCrops( results_vv,
		       adc_v,
		       ev_chstatus,
		       stitched_output_v );
    
  }

  /**
   * process cropped adc images through infill network
   *
   * @param[in] cropped_adc_v cropped adc images, ordered intp sets of 3 -- one for each plane --
   *                          expected to be made by ublarcvapp:UBSplitDetector.
   * @param[in] run run number
   * @param[in] subrun subrun number
   * @param[in] event event number
   * @param[inout] results_vv contains vector of images for each plane
   * @param[in] debug if true, run server functions in debug mode.
   */
  void Infill::processSparseCroppedInfillViaServer( const std::vector<std::vector<larcv::SparseImage> >& cropped_vv,
						    const int run, const int subrun, const int event, 
						    std::vector<std::vector<larcv::SparseImage> >& results_vv,
						    const float threshold,
						    bool debug ) {
    
    int replies_to_input_ratio = 1;

    results_vv.clear();
    results_vv.reserve(cropped_vv.size());

    // send data to network
    for ( size_t plane=0; plane<cropped_vv.size(); plane++ ) {
      auto const& plane_cropped_v = cropped_vv.at(plane);

      char service_name[50];
      sprintf( service_name, "infill_plane%d", (int)plane );

      if ( debug )
	std::cout << "SEND images to Infill service, \"" << service_name << "\""
		  << ": " << plane_cropped_v.size() << std::endl;

      std::vector< std::vector<larcv::SparseImage> > netout_vv;
      ServerInterface::sendReceiveSparseImageData( service_name,
						   plane_cropped_v,
						   netout_vv,
						   run, subrun, event,
						   replies_to_input_ratio,
						   debug );

      if ( debug ) 
	std::cout << "RECEIVED " << netout_vv.size() << " replies" << std::endl;

      for ( auto& netout_v : netout_vv ) {
	for ( auto& netout : netout_v ) {
	  results_vv.at(plane).emplace_back( std::move(netout) );
	}
      }

    }//end of plane loop

    
  }

  void Infill::stitchSparseCrops( const std::vector< std::vector<larcv::SparseImage> >& netout_vv,
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

}
