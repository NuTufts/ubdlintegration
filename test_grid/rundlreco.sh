#!/bin/bash

# OUTPUT FILES FROM PREVIOUS STAGE
source /cvmfs/uboone.opensciencegrid.org/products/setup_uboone.sh

echo "FILES available"
ls -lh

SUPERA=out_larcv_test.root  # has adc image, chstatus, ssnet output, mrcnn
OPRECO=larlite_opreco.root
RECO2D=larlite_reco2d.root

# HERE's OUR HACK: bring down ubdl, bring up dllee_unified
unsetup ubdl

echo "<<< SETUP DLLEE_UNIFIED >>>"
setup dllee_unified v1_0_0 -q e17:prof

# SETUP ENV FOR TAGGER BIN
export PATH=$LARLITECV_BASEDIR/app/TaggerCROI/bin:$PATH

#echo "<<< CHECK ENV AFTER DLLEE_UNIFIED >>>"
#export 

echo "<<< PRIMARY CHAIN >>>"
echo "< RUN TAGGER >"
TAGGER_CONFIG=$DLLEE_UNIFIED_DIR/dlreco_scripts/tagger_configs/tagger_overlay_v2_splity.cfg
ls out_larcv_test.root > input_larcv.txt
ls larlite_opreco.root > input_larlite.txt
run_tagger $TAGGER_CONFIG

TAGGER_LARCV=tagger_anaout_larcv.root
TAGGER_LARLITE=tagger_anaout_larlite.root


echo "< RUN VERTEXER >"
VERTEX_CONFIG=$DLLEE_UNIFIED_DIR/dlreco_scripts/vertex_configs/prod_fullchain_mcc9ssnet_combined_newtag_extbnb_c10_union.cfg
#VERTEX_CONFIG=prod_fullchain_mcc9ssnet_combined_newtag_extbnb_c10_union.cfg # for debug
python $DLLEE_UNIFIED_DIR/dlreco_scripts/bin/run_vertexer.py -c $VERTEX_CONFIG -a vertexana.root -o vertexout.root -d ./ $SUPERA $TAGGER_LARCV
VERTEXOUT=vertexout.root
VERTEXANA=vertexana.root

echo "< RUN TRACKER >"
TRACKER_CONFIG=$DLLEE_UNIFIED_DIR/dlreco_scripts/tracker_configs/tracker_read_cosmo.cfg
python $DLLEE_UNIFIED_DIR/dlreco_scripts/bin/run_tracker_reco3d.py -c $TRACKER_CONFIG -i $SUPERA -t $TAGGER_LARCV -p $VERTEXOUT -d ./ 
TRACKEROUT=tracker_reco.root
TRACKERANA=tracker_anaout.root
mv -f tracker_reco_0.root $TRACKEROUT
mv -f tracker_anaout_0.root  $TRACKERANA

echo "<<< TRACKONLY CHAIN >>>"
echo "< RUN TRACKONLY VERTEXER >"
TRKONLY_VERTEX_CONFIG=$DLLEE_UNIFIED_DIR/dlreco_scripts/vertex_configs/prod_fullchain_alltracklabel_combined_newtag_extbnb_c10_union.cfg
#TRKONLY_VERTEX_CONFIG=prod_fullchain_alltracklabel_combined_newtag_extbnb_c10_union.cfg # for debug
python $DLLEE_UNIFIED_DIR/dlreco_scripts/bin/run_vertexer.py -c $TRKONLY_VERTEX_CONFIG -a vertexana_trackonly_temp.root -o vertexout_trackonly.root -d ./ $SUPERA $TAGGER_LARCV
TRKONLY_VERTEXOUT=vertexout_trackonly.root
TRKONLY_VERTEXANA=vertexana_trackonly.root
python $DLLEE_UNIFIED_DIR/dlreco_scripts/bin/rename_vertexana.py vertexana_trackonly_temp.root $TRKONLY_VERTEXANA

echo "< RUN TRACKONLY TRACKER >"
TRKONLY_TRACKER_CONFIG=$DLLEE_UNIFIED_DIR/dlreco_scripts/tracker_configs/tracker_read_cosmo_trackonlyvertexer.cfg
#TRKONLY_TRACKER_CONFIG=tracker_read_cosmo_trackonlyvertexer.cfg
python $DLLEE_UNIFIED_DIR/dlreco_scripts/bin/run_tracker_reco3d.py -c $TRKONLY_TRACKER_CONFIG -i $SUPERA -t $TAGGER_LARCV -p $TRKONLY_VERTEXOUT -d ./ 
TRKONLY_TRACKEROUT=tracker_reco_trackonly.root
TRKONLY_TRACKERANA=tracker_anaout_trackonly.root
mv -f tracker_reco_0.root    $TRKONLY_TRACKEROUT
mv -f tracker_anaout_0.root  $TRKONLY_TRACKERANA

echo "<< combine larlite files >>"
python $DLLEE_UNIFIED_DIR/dlreco_scripts/bin/combine_larlite.py -o larlite_dlmerged.root larlite_opreco.root larlite_reco2d.root tagger_anaout_larlite.root tracker_reco.root tracker_reco_trackonly.root 
echo "<<< HADD ROOT FILES >>>"
hadd -f merged_dlreco.root $VERTEXOUT $TRKONLY_VERTEXOUT $VERTEXANA $TRKONLY_VERTEXANA $TRACKERANA $TRKONLY_TRACKERANA larlite_dlmerged.root
echo "<<< Append UBDL Products >>>"
python $DLLEE_UNIFIED_DIR/dlreco_scripts/bin/append_ubdlproducts.py merged_dlreco.root out_larcv_test.root


