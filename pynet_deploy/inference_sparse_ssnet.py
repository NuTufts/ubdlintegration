from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os,sys
from ctypes import c_int
import numpy as np
import torch
import traceback

# sparse uresnet imports (in networks/sparse_ssnet)
import uresnet
from uresnet.flags      import URESNET_FLAGS
from uresnet.main_funcs import inference
from uresnet.trainval   import trainval

"""
Implements worker for SLAC's sparse uresnet
"""

def forwardpass( plane, nrows, ncols, sparse_bson_list, weights_filepath ):

    from ROOT import std
    from larcv import larcv
    larcv.json.load_jsonutils()
    
    print("[SparseSSNet] forwardpass")
    # Get Configs going:
    # configuration from Ran:
    """
    inference --full -pl 1 -mp PATH_TO_Plane1Weights-13999.ckpt -io larcv_sparse 
    -bs 64 -nc 5 -rs 1 -ss 512 -dd 2 -uns 5 -dkeys wire,label 
    -mn uresnet_sparse -it 10 -ld log/ -if PATH_TO_INPUT_ROOT_FILE
    """
    print("[SparseSSNet] create uresnet_flags object")
    try:
        sys.argv = ["inference_sparse_ssnet.py"]
        config = uresnet.flags.URESNET_FLAGS()
    except Exception as e:
        print("[SparseSSNet] error creating URESNET_FLAGS: {}".format(sys.exc_info()[0]))
        print("[SparseSSNet] trackback")
        print(traceback.format_exc())
        return None
    print("[SparseSSNet] set options dictionary")
    args = { "full":True,               # --full
             "plane":plane,        # -pl
             "model_path":weights_filepath,  # -mp
             "io_type":"larcv_sparse",  # -io
             "batch_size":1,            # -bs
             "num_class":5,             # -nc
             "uresnet_filters":16,      # -uf
             "report_step":1,           # -rs
             "spatial_size":512,        # -ss
             "data_dim":2,              # -dd
             "uresnet_num_strides": 6,  # -uns
             "data_keys":"wire,label",  # -dkeys
             "model_name":"uresnet_sparse", # -mn
             "iteration":1,            # -it
             "log_dir":"./log/",          # -ld
             "input_file":"none" }      # -if

    print("[SparseSSNet] update/set config")
    config.update(args)
    config.SPATIAL_SIZE = (nrows,ncols)
    config.TRAIN = False
        
    print("\n\n-- CONFIG --")
    for name in vars(config):
        attribute = getattr(config,name)
        if type(attribute) == type(config.parser): continue
        print("%s = %r" % (name, getattr(config, name)))

    # Set random seed for reproducibility
    #np.random.seed(config.SEED)
    #torch.manual_seed(config.SEED)

    print("[SparseSSNet] create trainval interface")
    interface = trainval(config)
    print("[SparseSSNet] initialize")
    interface.initialize()
    print("Loaded sparse pytorch_uresnet plane={}".format(plane))

    # parse the input data, loop over pybyte objects
    sparsedata_v = []
    rseid_v = []
    npts_v  = []
    ntotalpts = 0

    try:
        for bson in sparse_bson_list:
            c_run    = c_int()
            c_subrun = c_int()
            c_event  = c_int()
            c_id     = c_int()

            imgdata = larcv.json.sparseimg_from_bson_pybytes(bson,
                                                             c_run, 
                                                             c_subrun, 
                                                             c_event, 
                                                             c_id )
            npts = int(imgdata.pixellist().size()/(imgdata.nfeatures()+2))
            ntotalpts += npts
            sparsedata_v.append(imgdata)
            npts_v.append( npts )
            rseid_v.append( (c_run.value, c_subrun.value, c_event.value, c_id.value) )
        
        # make batch array
        batch_np = np.zeros( ( ntotalpts, 4 ) )
        startidx = 0
        idx      = 0
        for npts,img2d in zip( npts_v, sparsedata_v ):
            endidx   = startidx+npts
            spimg_np = larcv.as_ndarray( img2d, larcv.msg.kNORMAL )
            #print("spimg_np: {}".format(spimg_np[:,0:2]))

            # coords
            batch_np[startidx:endidx,0] = nrows-1-spimg_np[:,0] # tick
            batch_np[startidx:endidx,1] = spimg_np[:,1] # wire

            batch_np[startidx:endidx,2] = idx           # batch index
            batch_np[startidx:endidx,3] = spimg_np[:,2] # pixel value
            #print("batch_np: {}".format(batch_np[:,0:2]))
            idx += 1

        # pass to network
        data_blob = { 'data': [[batch_np]] }
        results = interface.forward( data_blob )
    except:
        print("[SparseSSNet] error converting msg/running net: {}".format(sys.exc_info()[0]))
        print("[SparseSSNet] trackback")
        print(traceback.format_exc())
        return None


    bson_reply = []
    startidx = 0
    try:
        for idx in xrange(len(results['softmax'])):
            ssnetout_np = results['softmax'][idx]
            #print("ssneout_np: {}".format(ssnetout_np.shape))
            rseid = rseid_v[idx]
            meta  = sparsedata_v[idx].meta(0)
            npts  = int( npts_v[idx] )
            endidx = startidx+npts
            #print("numpoints for img[{}]: {}".format(idx,npts))
            ssnetout_wcoords = np.zeros( (ssnetout_np.shape[0],ssnetout_np.shape[1]+2), dtype=np.float32 )
            
            ssnetout_wcoords[:,0] = nrows-1-batch_np[startidx:endidx,0] # tick
            ssnetout_wcoords[:,1] = batch_np[startidx:endidx,1] # wire
            
            # pixel value
            ssnetout_wcoords[:,2:2+ssnetout_np.shape[1]] = ssnetout_np[:,:]
            startidx = endidx
            #print("ssnetout_wcoords: {}".format(ssnetout_wcoords[:,0:2]))
            
            meta_v = std.vector("larcv::ImageMeta")()
            for i in xrange(5):
                meta_v.push_back(meta)
            
            ssnetout_spimg = larcv.sparseimg_from_ndarray( ssnetout_wcoords, meta_v, larcv.msg.kDEBUG )
            bson = larcv.json.as_bson_pybytes( ssnetout_spimg, rseid[0], rseid[1], rseid[2], rseid[3] )
                                          
            bson_reply.append(bson)
    except:
        print("[SparseSSNet] error packing up data for return: {}".format(sys.exc_info()[0]))
        print("[SparseSSNet] trackback")
        print(traceback.format_exc())
        return None
        

    return bson_reply
    
    
    
        
if __name__ == "__main__":

    print("Test Inference Sparse-Infill")
    import ROOT 
    from ROOT import std
    from larcv import larcv
    larcv.PSet
    larcv.json.load_jsonutils
    from ublarcvapp import ublarcvapp

    supera_file = sys.argv[1]
    io = larcv.IOManager(larcv.IOManager.kREAD,"supera",larcv.IOManager.kTickBackward)
    io.add_in_file( supera_file )
    io.initialize()

    weight_dir = "/uboone/data/users/tmw/dl_model_files"
    weights = [ weight_dir+"/Plane0Weights-13999.ckpt",
                weight_dir+"/Plane1Weights-17999.ckpt",
                weight_dir+"/Plane2Weights-26999.ckpt" ]


    nentries = io.get_n_entries()

    for ientry in range(nentries):
        io.read_entry(ientry)

        # Event Image
        ev_img = io.get_data( larcv.kProductImage2D, "wire" )
        img_v  = ev_img.Image2DArray()

        results_v = []
        for p in [0,1,2]:
            pimg_v = std.vector("larcv::Image2D")()
            pimg_v.push_back( img_v.at(p) )

            sparseimg_v = []
            thresholds = std.vector("float")(1,10.0)

            sparseimg = larcv.SparseImage( pimg_v, thresholds )
        
            iset = 0
            bson = larcv.json.as_bson_pybytes( sparseimg, 
                                               io.event_id().run(),
                                               io.event_id().subrun(),
                                               io.event_id().event(), iset )
            sparseimg_v.append(bson)

            results = forwardpass( p, sparseimg_v, weights[p] )
            results_v.append( results )
    
        break
