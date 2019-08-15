#include "CosmicMRCNN.h"


namespace dl {

  /**
   * process whole images through ub mask-rcnn server
   *
   * @param[in] wholeview_adc_v cropped adc images, ordered intp sets of 3 -- one for each plane --
   *                          expected to be made by ublarcvapp:UBSplitDetector.
   * @param[in] run run number
   * @param[in] subrun subrun number
   * @param[in] event event number
   * @param[inout] result_mask_vv for each plane, a vector of clustermasks
   * @param[in] debug if true, run server functions in debug mode.
   */
  void CosmicMRCNN::processViaServer( const std::vector<larcv::Image2D>& wholeview_adc_v,
				      const int run, const int subrun, const int event, 
				      std::vector< std::vector<larcv::ClusterMask> >& result_mask_vv,
				      bool debug ) {
    
    // need to convert to larcv::SparseImage
    // std::vector< larcv::SparseImage > wholeview_sparse_v;
    // std::vector< float > thresholds_v( 1, threshold );
    // std::vector< int >   require_pixel( 1, 1 );

    // for ( auto conost& img : wholeview_adc_v ) {
    //   std::vector< const larcv::Image2D* > img_v;
    //   img_v.push_back( &img );
    //   larcv::SparseImage sparse( img_v, thresholds_v, require_pixel );
    //   wholeview_sparse_v.emplace_back( std::move(sparse) );
    // }
    
    result_mask_vv.clear();
    result_mask_vv.resize(wholeview_adc_v.size());

    for ( size_t p=0; p<wholeview_adc_v.size(); p++  ) {
      
      char service_name[50];
      sprintf( service_name, "ubmrcnn_plane%d", (int)p );

      std::vector<larcv::Image2D> input_v;
      input_v.push_back( wholeview_adc_v.at(p) );
      
      std::vector< std::vector< larcv::ClusterMask > > results_vv;
      ServerInterface::sendReceiveData<larcv::Image2D,larcv::ClusterMask>( service_name,
									   input_v,
									   results_vv,
									   run, subrun, event, 
									   0, debug );
      
      // pass to output container
      int nmasks = 0;
      for ( auto& mask_v : results_vv ) {
	nmasks += mask_v.size();
	for ( auto& mask : mask_v ) {
	  result_mask_vv[p].emplace_back( std::move(mask) );
	}
      }
      std::cout << __FUNCTION__ << ":" << __LINE__ << ":: saving " 
		<< nmasks << " images from " << service_name << std::endl;
    }//end of plane loop
    
  }
  
  

}
