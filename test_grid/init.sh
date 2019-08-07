#!/bin/bash

export LD_LIBRARY_PATH=${LIBZMQ_FQ_DIR}/lib64:${LD_LIBRARY_PATH}
#export PYTHONPATH=${PWD}:/pnfs/uboone/resilient/users/tmw/model_deploy_scripts:${PYTHONPATH}

ifdh cp /pnfs/uboone/resilient/users/tmw/model_deploy_scripts/inference_mrcnn.py inference_mrcnn.py
ifdh cp /pnfs/uboone/resilient/users/tmw/model_deploy_scripts/Infill_ForwardPass.py Infill_ForwardPass.py
export PYTHONPATH=${PWD}:${PYTHONPATH}

# larlite
#export ROOT_INCLUDE_PATH=$LARLITE_BASEDIR/core/Base:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$LARLITE_BASEDIR/core/Analysis:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$LARLITE_BASEDIR/core/DataFormat:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$LARLITE_BASEDIR/core/LArUtil:$ROOT_INCLUDE_PATH

# geo2d
#export ROOT_INCLUDE_PATH=$GEO2D_INCDIR/Geo2D/Core:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$GEO2D_INCDIR/Geo2D/Algorithm:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$GEO2D_INCDIR/Geo2D/PyUtil:$ROOT_INCLUDE_PATH

# laropencv
#export ROOT_INCLUDE_PATH=$LAROPENCV_BASEDIR/LArOpenCV/Core
#export ROOT_INCLUDE_PATH=$LAROPENCV_BASEDIR/LArOpenCV/App
#export ROOT_INCLUDE_PATH=$LAROPENCV_BASEDIR/LArOpenCV/ImageCluster/AlgoBase
#export ROOT_INCLUDE_PATH=$LAROPENCV_BASEDIR/LArOpenCV/ImageCluster/AlgoClass
#export ROOT_INCLUDE_PATH=$LAROPENCV_BASEDIR/LArOpenCV/ImageCluster/AlgoData
#export ROOT_INCLUDE_PATH=$LAROPENCV_BASEDIR/LArOpenCV/ImageCluster/AlgoFunction
#export ROOT_INCLUDE_PATH=$LAROPENCV_BASEDIR/LArOpenCV/ImageCluster/AlgoModule

# larcv
#export ROOT_INCLUDE_PATH=$LARCV_INCDIR/larcv/core/Base:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$LARCV_INCDIR/larcv/core/DataFormat:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$LARCV_INCDIR/larcv/core/CPPUtil:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$LARCV_INCDIR/larcv/core/CVUtil:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$LARCV_INCDIR/larcv/core/json:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$LARCV_INCDIR/larcv/core/Processor:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$LARCV_INCDIR/larcv/core/PyUtil:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$LARCV_INCDIR/larcv/core/ROOTUtil:$ROOT_INCLUDE_PATH
#export ROOT_INCLUDE_PATH=$LARCV_INCDIR/larcv/core/TorchUtil:$ROOT_INCLUDE_PATH

# ublarcvapp
#export ROOT_INCLUDE_PATH=$UBLARCVAPP_INCDIR/ublarcvapp/ContourTools
#export ROOT_INCLUDE_PATH=$UBLARCVAPP_INCDIR/ublarcvapp/dbscan
#export ROOT_INCLUDE_PATH=$UBLARCVAPP_INCDIR/ublarcvapp/Filter
#export ROOT_INCLUDE_PATH=$UBLARCVAPP_INCDIR/ublarcvapp/LArliteHandler
#export ROOT_INCLUDE_PATH=$UBLARCVAPP_INCDIR/ublarcvapp/LArOpenCVHandl
#export ROOT_INCLUDE_PATH=$UBLARCVAPP_INCDIR/ublarcvapp/Reco3D
#export ROOT_INCLUDE_PATH=$UBLARCVAPP_INCDIR/ublarcvapp/Stitchers
#export ROOT_INCLUDE_PATH=$UBLARCVAPP_INCDIR/ublarcvapp/Tagger
#export ROOT_INCLUDE_PATH=$UBLARCVAPP_INCDIR/ublarcvapp/ubdllee
#export ROOT_INCLUDE_PATH=$UBLARCVAPP_INCDIR/ublarcvapp/UBImageMod
#export ROOT_INCLUDE_PATH=$UBLARCVAPP_INCDIR/ublarcvapp/UBWireTool

# larflow
#export ROOT_INCLUDE_PATH=$LARFLOW_INCDIR/larflow/FlowContourMatch


#ldd $UBCV_LIB/*.so
#ldd $LARCV_LIBDIR/*.so

#echo "<<PYTHONPATH>>"
#echo $PYTHONPATH | sed 's|:|\n|g'
#echo "<<LD_LIBRARY_PATH>>"
#echo $LD_LIBRARY_PATH | sed 's|:|\n|g'
#ls

#echo "<< INFERENCE_MRCNN test >>"
#python -c "import inference_mrcnn"

#echo "<< INFILL test >>"
#python -c "import Infill_ForwardPass"

#echo "<< larcv.json test >>"
#python -c "from larcv import larcv; larcv.json.load_jsonutils(); print \"larcv.json.load_jsonutils()\""

echo "<< mrcnn run test >>"
ifdh cp /pnfs/uboone/resilient/users/tmw/model_configs/ubmrcnn_mcc8_cfgs/mills_config_0.yaml mills_config_0.yaml
ifdh cp /pnfs/uboone/resilient/users/tmw/model_configs/ubmrcnn_mcc8_cfgs/mills_config_1.yaml mills_config_1.yaml
ifdh cp /pnfs/uboone/resilient/users/tmw/model_configs/ubmrcnn_mcc8_cfgs/mills_config_2.yaml mills_config_2.yaml
#ifdh cp /pnfs/uboone/resilient/users/tmw/model_configs/ubmrcnn_mcc8_cfgs/supera-Run005121-SubRun000004.root supera-Run005121-SubRun000004.root
#python inference_mrcnn.py supera-Run005121-SubRun000004.root
