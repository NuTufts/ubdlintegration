#include "LArFlow.h"

#include "larflow/FlowContourMatch/FlowContourMatch.h"

namespace dl {

  /**
   * process cropped adc images through dual flow larflow.
   *
   * @param[in] cropped_adc_v cropped adc images, ordered intp sets of 3 -- one for each plane --
   *                          expected to be made by ublarcvapp:UBSplitDetector.
   * @param[in] run run number
   * @param[in] subrun subrun number
   * @param[in] event event number
   * @param[inout] flowresults_v flows for each cropped image set, ordered by Y2U and then Y2V.
   * @param[in] debug if true, run server functions in debug mode.
   */
  void LArFlow::processCroppedLArFlowViaServer( const std::vector<larcv::Image2D>& cropped_adc_v,
						const int run, const int subrun, const int event, const float threshold,
						std::vector<larcv::SparseImage>& flowresults_v,
						bool debug ) {
    
    // need to convert to larcv::SparseImage
    std::vector< larcv::SparseImage > cropped_sparse_v;
    std::vector< float > thresholds_v( 3, threshold );
    std::vector< int >   require_pixel( 3, 1 );

    int nsets = cropped_adc_v.size()/3;
    for ( int iset=0; iset<nsets; iset++ ) {
      std::vector< const larcv::Image2D* > img_v;
      for ( size_t p=0; p<3; p++ ) {
	img_v.push_back( &cropped_adc_v.at( 3*iset+p ) );
      }
      larcv::SparseImage sparse( img_v, thresholds_v, require_pixel );
      cropped_sparse_v.emplace_back( std::move(sparse) );
    }

    std::vector< std::vector< larcv::SparseImage > > results_vv;
    
    ServerInterface::sendReceiveSparseImageData( "ublarflow_plane2", 
						 cropped_sparse_v,
						 results_vv,
						 run, subrun, event, 2, debug );
    
    // convert back to image2d
    for ( auto const& sparse_v : results_vv ) {
      for ( auto const& sparse : sparse_v ) {
	flowresults_v.emplace_back( std::move(sparse) );
      }
    }
    
    std::cout << "saving " << flowresults_v.size() << " images from sparse-larflow-server" << std::endl;
  }

  /**
   * convert larflow result into larlite hits
   *
   */
  std::vector<larlite::larflow3dhit> 
  LArFlow::croppedFlow2hits( const std::vector<larcv::SparseImage>& flowresults_v, 
			     const std::vector<larcv::Image2D>& wholeview_v,
			     const std::string ubcroptrueflow_cfg,
			     const float threshold,
			     bool debug ) {

    larcv::msg::Level_t verbosity = larcv::msg::kNORMAL;
    if ( debug )
      verbosity = larcv::msg::kDEBUG;
    
    std::vector<larlite::larflow3dhit> hit_v
      = larflow::makeFlowHitsFromSparseCrops( wholeview_v, flowresults_v,
					      threshold, ubcroptrueflow_cfg, 
					      verbosity );
    return hit_v;
  }


}
