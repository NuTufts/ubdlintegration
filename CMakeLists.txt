set( LARLITE_LIB -L$ENV{LARLITE_LIBDIR} -lLArLite_Analysis -lLArLite_LArUtil -lLArLite_DataFormat -lLArLite_Base )

set( UBOONECODE_LIB -L$ENV{UBOONECODE_LIB} )

include_directories ( $ENV{LARLITE_COREDIR} ${LARCV_INCLUDE_DIR} $ENV{UBLARCVAPP_INCDIR} $ENV{LARFLOW_INCDIR} ${PYTHON_INCLUDE_DIRS} ${ZeroMQ_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
# FOR DEBUG
#message("python ${PYTHON_INCLUDE_DIRS}")
#message("python ${PYTHON_LIBRARIES}")
message("opencv: ${OpenCV_INCLUDE_DIRS}")
#message("ZMQ lib: ${ZeroMQ_LIBRARY}")
#message("ublarcvapp LIBS: ${ublarcvapp_LIBRARIES}")
#message("LARFLOW LIBS: ${larflow_LIBRARIES}")
#message("LARFLOW INC: $ENV{LARFLOW_INCDIR}")

set(CMAKE_CXX_STANDARD 17)
if (Torch_FOUND)
  include_directories ( ${TORCH_INCLUDE_DIRS} )
  link_libraries( ${LIB_NAME} -L$ENV{BOOST_LIB} -lboost_system ${ROOTLIB} ${LARLITE_LIB} ${ublarcvapp_LIBRARIES} ${larflow_LIBRARIES} ${TORCH_LIBRARIES} ${PYTHON_LIBRARIES} ) 
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DHAS_TORCH")
else()
  link_libraries( ${LIB_NAME} -L$ENV{BOOST_LIB} -lboost_system ${ROOTLIB} ${LARLITE_LIB} ${ublarcvapp_LIBRARIES} ${larflow_LIBRARIES} ${PYTHON_LIBRARIES}) 
endif()
add_definitions("-Wno-register")
add_definitions("-DHASPYUTIL")

art_make( BASENAME_ONLY
	  LIBRARY_NAME     ubcv_ubdlintegration
	  LIB_LIBRARIES
	  	   larcorealg_Geometry
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
                   art_Persistency_Common
                   art_Persistency_Provenance
		   art_Utilities canvas
		   ${MF_MESSAGELOGGER}
		   
		   ${FHICLCPP}
		   cetlib cetlib_except
		   LArFlow_FlowContourMatch
		   LArCVCoreBase
		   LArCVCoreDataFormat
		   LArCVCoreProcessor
		   LArCVCoreJSON
		   LArCVCorePyUtil
		   ${ROOT_GEOM}
		   ${ROOT_XMLIO}
		   ${ROOT_GDML}
		   ${ROOT_BASIC_LIB_LIST}
		   ${ZeroMQ_LIBRARY}

          MODULE_LIBRARIES 	  	
	  	   ubcv_ubdlintegration
	  	   uboonecode_uboone_LArCVImageMaker
		   ubcv_ubdldata
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
                   art_Persistency_Common
                   art_Persistency_Provenance
		   art_Utilities canvas
		   ${MF_MESSAGELOGGER}

		   #${LARCV_LIB} 
		   ${LARLITE_LIB} 
		   #${TORCH_LIBRARIES}
		   
		   ${FHICLCPP}
		   cetlib_except
		   ${ROOT_GEOM}
		   ${ROOT_XMLIO}
		   ${ROOT_GDML}
		   ${ROOT_BASIC_LIB_LIST}
		   ${ZeroMQ_LIBRARY}
		)

if (Torch_FOUND)
   target_link_libraries( DLInterface_module cppzmq ${ZeroMQ_LIBRARY} LArCVCoreBase LArCVCoreDataFormat LArCVCoreProcessor LArCVCoreJSON LArCVCoreTorchUtil LArCVApp_UBImageMod LArCVApp_ubdllee LArFlow_FlowContourMatch)
else()
   target_link_libraries( DLInterface_module cppzmq ${ZeroMQ_LIBRARY} LArCVCoreBase LArCVCoreDataFormat LArCVCoreProcessor LArCVCoreJSON LArCVApp_UBImageMod LArCVApp_ubdllee LArFlow_FlowContourMatch)
endif()


# install_headers()
install_fhicl()
install_source()
