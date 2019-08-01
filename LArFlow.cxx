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
    
    // Sent message format differs for SPARSE and DENSE network
    
    if ( _net_type==kSPARSE ) {

      std::vector< larcv::SparseImage > cropped_sparse_v;
      std::vector< std::vector< larcv::SparseImage > > results_vv;
    
      std::vector< float > thresholds_v( 3, threshold );
      std::vector< int >   require_pixel( 3, 1 );
      
      int nsets = cropped_adc_v.size()/3;
      for ( int iset=0; iset<nsets; iset++ ) {
	std::vector< const larcv::Image2D* > img_v;
	img_v.push_back( &cropped_adc_v.at( 3*iset+ 2 ) ); // src-Y first
	img_v.push_back( &cropped_adc_v.at( 3*iset+ 0 ) ); // tar1-U
	img_v.push_back( &cropped_adc_v.at( 3*iset+ 1 ) ); // tar2-U
	//std::cout << "SparseImage source Meta set[" << iset << "]: " << img_v.at(0)->meta().dump() << std::endl;
	larcv::SparseImage sparse( img_v, thresholds_v, require_pixel );
	cropped_sparse_v.emplace_back( std::move(sparse) );
      }


    
      // ServerInterface::sendReceiveSparseImageData( "ublarflow_plane2", 
      // 						 cropped_sparse_v,
      // 						 results_vv,
      // 						 run, subrun, event, 2, debug );
      ServerInterface::sendReceiveData<larcv::SparseImage,larcv::SparseImage>( "ublarflow_plane2", 
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
    else if ( _net_type==kDENSE ) {

      // we have two sets, one for each flow direction: Y2U and Y2V
      enum { kY2U=0, kY2V };

      std::vector< larcv::SparseImage > cropped_sparse_v[2];
      std::vector< std::vector< larcv::SparseImage > > results_vv[2];

      std::vector< float > thresholds_v( 1, threshold );
      std::vector< int >   require_pixel( 1, 1 );
      
      // loopp over sets
      int nsets = cropped_adc_v.size()/3;
      for ( int iset=0; iset<nsets; iset++ ) {
	// sparsify each individually	
	std::vector< const larcv::Image2D* > src_img_v;
	src_img_v.push_back( &cropped_adc_v.at( 3*iset+ 2 ) ); // src-Y first
	larcv::SparseImage sparse_src( src_img_v, thresholds_v, require_pixel );

	std::vector< const larcv::Image2D* > tary2u_img_v;
	tary2u_img_v.push_back( &cropped_adc_v.at( 3*iset+ 0 ) ); // tar1-U
	larcv::SparseImage sparse_tary2u( tary2u_img_v, thresholds_v, require_pixel );

	std::vector< const larcv::Image2D* > tary2v_img_v;	
	tary2v_img_v.push_back( &cropped_adc_v.at( 3*iset+ 1 ) ); // tar2-U
	larcv::SparseImage sparse_tary2v( tary2v_img_v, thresholds_v, require_pixel );


	cropped_sparse_v[kY2U].push_back( sparse_src );
	cropped_sparse_v[kY2U].push_back( sparse_tary2u );

	cropped_sparse_v[kY2V].push_back( sparse_src );
	cropped_sparse_v[kY2V].push_back( sparse_tary2v );
	
      }


    
      std::cout << "SEND Y2U" << std::endl;
      debug = true;
      ServerInterface::sendReceiveData<larcv::SparseImage,larcv::SparseImage>( "ublarflow_dense_y2u", 
									       cropped_sparse_v[kY2U],
									       results_vv[kY2U],
									       run, subrun, event, 0, debug );
      std::cout << "SEND Y2V" << std::endl;
      ServerInterface::sendReceiveData<larcv::SparseImage,larcv::SparseImage>( "ublarflow_dense_y2v", 
									       cropped_sparse_v[kY2V],
									       results_vv[kY2V],
									       run, subrun, event, 0, debug );
    
      // merge the output back together
      int ny2u = results_vv[kY2U].size();
      int ny2v = results_vv[kY2V].size();
      std::vector<float> thresh_reply_v(3,10);
      std::vector<int>   require_reply_pix_v(3,0);
      require_reply_pix_v[0] = 1;

      if ( ny2u!=ny2v ) {
	// ERROR
	std::stringstream msg;
	msg << "LArFlow::processCroppedLArFlowViaServer:" << __LINE__ 
	    << " number of Y2U and Y2V crops do not match: " << ny2u << "!=" << ny2v  << "."
	    << " expected " << cropped_sparse_v[kY2U].size()/2 << " replies" << std::endl;
	throw std::runtime_error( msg.str() );
      }

      for (int ireply=0; ireply<ny2u; ireply++ ) {
	
	larcv::SparseImage& reply_y2u = results_vv[kY2U].at(ireply).front();
	larcv::SparseImage& reply_y2v = results_vv[kY2V].at(ireply).front();
	
	// redensify
	larcv::Image2D dense_y2u = reply_y2u.as_Image2D().front();
	larcv::Image2D dense_y2v = reply_y2v.as_Image2D().front();

	// sparify with source image
	std::vector<const larcv::Image2D*> combine_v;
	combine_v.push_back( &cropped_adc_v.at( 3*ireply+ 2 ) ); // 
	combine_v.push_back( &dense_y2u );
	combine_v.push_back( &dense_y2v );
	
	larcv::SparseImage sparse_reply( combine_v, thresh_reply_v, require_reply_pix_v );
	
	flowresults_v.emplace_back( std::move(sparse_reply) );
	
      }// loop over reply
      
    }//if dense
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

    if ( debug )
      std::cout << __FILE__ << "." << __LINE__ << ": Num of input sparse images containing flow prediction: " << flowresults_v.size() << std::endl;

    // if ( flowresults_v.size()==66 ) {
    //   // hackish -- we have to split the sparseimage up unfortunately
    //   std::vector<larcv::SparseImage> flow_v;
    //   for ( auto const& spimg : flowresults_v ) {
    // 	size_t npts = spimg.pixellist().size()/4;
    // 	std::vector<float> y2u_v( npts*3, 0 );
    // 	std::vector<float> y2v_v( npts*3, 0 );
    // 	for ( size_t idx=0; idx<npts; idx++ ) {
    // 	  for (int i=0; i<2; i++ ) {
    // 	    y2u_v[idx*3+i] = spimg.pixellist()[ idx*4+i ];
    // 	    y2v_v[idx*3+i] = spimg.pixellist()[ idx*4+i ];
    // 	  }
    // 	  y2u_v[idx*3+2] = spimg.pixellist()[ idx*4+2 ];
    // 	  y2v_v[idx*3+2] = spimg.pixellist()[ idx*4+3 ];
    // 	}

    // 	std::vector<larcv::ImageMeta> meta_y2u;
    // 	std::vector<larcv::ImageMeta> meta_y2v;
    // 	meta_y2u.push_back( spimg.meta(0) );
    // 	meta_y2v.push_back( spimg.meta(1) );

    // 	larcv::SparseImage y2u( 1, npts, y2u_v, meta_y2u );
    // 	larcv::SparseImage y2v( 1, npts, y2v_v, meta_y2v );
    // 	flow_v.emplace_back( y2u );
    // 	flow_v.emplace_back( y2v );
    //   }
    //   if ( debug )
    // 	std::cout << __FILE__ << "." << __LINE__ << ": split dual flow into sparse image sets: " << flow_v.size() << std::endl;

    //   std::vector<larlite::larflow3dhit> hit_v;
    //   try {
    // 	hit_v = larflow::makeFlowHitsFromSparseCrops( wholeview_v, flow_v,
    // 						      threshold, ubcroptrueflow_cfg, 
    // 						      verbosity );
    //   }
    //   catch ( std::exception& e ) {
    // 	std::cout << __FILE__ << "." << __LINE__ << ": caught exception while making flow hits\n" 
    // 		  << "   " 
    // 		  << e.what()
    // 		  << std::endl;
    // 	// pass exception along
    // 	throw std::exception(e);
    //   }
    //   return hit_v;
    // }
    // else {

    std::vector<larlite::larflow3dhit> hit_v;
    try {
      hit_v = larflow::makeFlowHitsFromSparseCrops( wholeview_v, flowresults_v,
						    threshold, ubcroptrueflow_cfg, 
						    verbosity );
    }
    catch ( const std::exception& e ) {
      std::cout << __FILE__ << "." << __LINE__ << ": caught exception while making flow hits\n" 
		<< "   " 
		<< e.what()
		<< std::endl;
      // pass exception along
      throw e;
    }
    
    return hit_v;
    //}
    //return std::vector<larlite::larflow3dhit>();
  }


}
