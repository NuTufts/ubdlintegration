set( LARCV_LIB -L$ENV{LARCV_LIBDIR} -llarcv )
set( LARLITE_LIB -L$ENV{LARLITE_LIBDIR} -lLArLite_Analysis -lLArLite_LArUtil -lLArLite_DataFormat -lLArLite_Base )
set( ZMQ_LIB -L/usr/local/lib -lzmq )

set( UBOONECODE_LIB -L$ENV{UBOONECODE_LIB} )

include_directories ( $ENV{LARCV_COREDIR} $ENV{LARCV_APPDIR} $ENV{LARLITE_COREDIR} )

link_libraries( ${LIB_NAME} -L$ENV{BOOST_LIB} -lboost_system ${ROOTLIB} ${LARCV_LIB} ${LARLITE_LIB} ${ZMQ_LIB} ) # All execs need this.

art_make( BASENAME_ONLY
	  LIBRARY_NAME     uboonecode_uboone_ubdlintegration
	  LIB_LIBRARIES
	           larcorealg_Geometry
	           uboonecode_uboone_LArCVImageMaker	   
		   ubcore_Geometry
		   ubobj_MuCS
		   larcore_Geometry_Geometry_service
	   	   nutools_ParticleNavigation lardataobj_Simulation
		   lardataobj_RecoBase
		   nusimdata_SimulationBase
		   lardataobj_MCBase
		   larcoreobj_SummaryData
		   lardataobj_RawData
		   lardata_RecoObjects
		   ${UBOONECODE_LIB}
		   nusimdata_SimulationBase
		   ${LARRECO_LIB}
		   ${LARDATA_LIB}
		   ${ART_FRAMEWORK_CORE}
		   ${ART_FRAMEWORK_PRINCIPAL}
                   ${ART_FRAMEWORK_BASIC}
		   ${ART_FRAMEWORK_SERVICES_REGISTRY}
		   ${ART_FRAMEWORK_SERVICES_OPTIONAL}
		   ${ART_FRAMEWORK_SERVICES_OPTIONAL_TFILESERVICE_SERVICE}
		   art_Persistency_Common canvas
		   art_Persistency_Provenance canvas
		   art_Utilities canvas
		   ${MF_MESSAGELOGGER}
		   
		   ${FHICLCPP}
		   cetlib cetlib_except
		   ${ROOT_GEOM}
		   ${ROOT_XMLIO}
		   ${ROOT_GDML}
		   ${ROOT_BASIC_LIB_LIST}

		   MODULE_LIBRARIES
		   uboonecode_uboone_LArCVImageMaker
	      	   TFileMetadataMicroBooNE_service
	      	   FileCatalogMetadataMicroBooNE_service
		   LArCVMetaMaker_service
		   ubobj_MuCS
		   larcorealg_Geometry
		   ubcore_Geometry
		   larcore_Geometry_Geometry_service
		   nutools_ParticleNavigation lardataobj_Simulation
                   larevt_CalibrationDBI_IOVData
                   larevt_CalibrationDBI_Providers
		   nusimdata_SimulationBase
		   lardata_Utilities
		   lardataobj_MCBase
		   larcoreobj_SummaryData
		   lardataobj_RawData
		   lardataobj_RecoBase
		   lardata_RecoObjects
                   larevt_Filters
                   larevt_CalibrationDBI_IOVData
                   larevt_CalibrationDBI_Providers
		   ${UBOONECODE_LIB}
		   nusimdata_SimulationBase
		   ${LARRECO_LIB}
		   ${LARDATA_LIB}
		   ${ART_FRAMEWORK_CORE}
		   ${ART_FRAMEWORK_PRINCIPAL}
                   ${ART_FRAMEWORK_BASIC}
		   ${ART_FRAMEWORK_SERVICES_REGISTRY}
		   ${ART_FRAMEWORK_SERVICES_OPTIONAL}
		   ${ART_FRAMEWORK_SERVICES_OPTIONAL_TFILESERVICE_SERVICE}
		   art_Persistency_Common canvas
		   art_Persistency_Provenance canvas
		   art_Utilities canvas
		   ${MF_MESSAGELOGGER}
		   
		   ${FHICLCPP}
		   cetlib_except
		   ${ROOT_GEOM}
		   ${ROOT_XMLIO}
		   ${ROOT_GDML}
		   ${ROOT_BASIC_LIB_LIST}
		)

# simple_plugin(LArCVMetaMaker "service"
# 	      TFileMetadataMicroBooNE_service
# 	      FileCatalogMetadataMicroBooNE_service
# 	      ${ART_FRAMEWORK_SERVICES_SYSTEM_FILECATALOGMETADATA_SERVICE}	      
# 	      art_Utilities canvas_Utilities
# 	      ${MF_MESSAGELOGGER}
# 	      ${MF_UTILITIES}
# 	      ${FHICLCPP}
# 	      ${CETLIB}
# 	      #cetlib cetlib_except
# 	      ${ROOT_BASIC_LIB_LIST}
#               BASENAME_ONLY
# )

# install_headers()
install_fhicl()
install_source()
