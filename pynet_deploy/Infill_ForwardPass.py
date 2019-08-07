#!/bin/env python
from __future__ import print_function
## IMPORT


# python,numpy
import os,sys,commands
import shutil
import time
import traceback
from ctypes import c_int
import numpy as np

# torch
import torch


# sparse model
from sparseinfill import SparseInfill

def forwardpass( sparseimg_bson_list, checkpoint_file ):
    """ function to load the sparse infill network and run a forward pass of one image
    make tensor for coords (row,col,batch). expects an input consisting of a list of pybyte 
    objects containing json versions of SparseImage """
    print("[INFILL] forward pass")

    # import larcv
    print("[INFILL] Load modules: ROOT, larcv, ublarcvapp, jsonutils")

    print("[INFILL] Load modules: ROOT")
    from ROOT import std
    print("[INFILL] Load modules: larcv")
    from larcv import larcv
    print("[INFILL] Load modules: ublarcapp")
    from ublarcvapp import ublarcvapp
    print("[INFILL] Load modules: load_jsonutils")
    larcv.json.load_jsonutils

    batchsize = 1
    starttime = time.time()

    # load the model
    print("[INFILL] define sparse model")
    model = SparseInfill( (512,496), 1,16,16,5, show_sizes=False)

    # load checkpoint data
    print("[INFILL] load checkpoint file {}".format(checkpoint_file))
    checkpoint = torch.load( checkpoint_file, {"cuda:0":"cpu","cuda:1":"cpu"} )
    best_prec1 = checkpoint["best_prec1"]
    model.load_state_dict(checkpoint["state_dict"])
    loadedmodeltime = time.time()-starttime
    print("[INFILL] model loading time: %.2f secs"%(loadedmodeltime))
    
    # parse the input data, loop over pybyte objects
    sparsedata_v = []
    rse_v = []
    ntotalpts = 0
    for bson in sparseimg_bson_list:
        c_run    = c_int()
        c_subrun = c_int()
        c_event  = c_int()
        c_id     = c_int()

        imgdata = larcv.json.sparseimg_from_bson_pybytes(bson,
                                                         c_run, 
                                                         c_subrun, 
                                                         c_event, 
                                                         c_id )

        sparsedata_v.append(imgdata)
        rse_v.append( (c_run.value, c_subrun.value, c_event.value, c_id.value) )

    nbatches = len(sparsedata_v)/batchsize
    if len(sparsedata_v)%batchsize!=0:
        nbatches += 1

    iimgs = 0
    tloadbatchdata = 0.
    trunmodel = 0.
    tpackoutput = 0.
    
    bson_results_v = []

    # loop over batches
    for ibatch in range(nbatches):

        starttime = time.time()

        # count the number of pts in each image of the batch
        totalpts = 0
        npts_v   = []
        for ib in range(batchsize):
            img_idx = ibatch*batchsize + ib
            if img_idx>=len(sparsedata_v):
                continue

            img = sparsedata_v[img_idx]
            imgpts   = int(img.pixellist().size()/(img.nfeatures()+2))
            print("pixlist={} nfeatures={} npts={}".format( img.pixellist().size(), img.nfeatures(), imgpts ))
            totalpts += imgpts
            npts_v.append( imgpts )
        print("Pt totals: ",npts_v)

        # prepare numpy array for batch
        coord_np = np.zeros( (totalpts,3), dtype=np.int )
        input_np = np.zeros( (totalpts,1), dtype=np.float32 )
        
        # fill data into batch arrays
        nfilled = 0
        for ib in range(batchsize):
            img_idx  = ibatch*batchsize + ib
            if img_idx>=len(sparsedata_v):
                continue

            img_np = larcv.as_ndarray( sparsedata_v[img_idx], larcv.msg.kNORMAL )
            start  = nfilled
            end    = nfilled+npts_v[ib]

            coord_np[start:end,0:2] = img_np[:,0:2].astype(np.int)
            coord_np[start:end,2]   = ib
            input_np[start:end,0]   = img_np[:,2]
            nfilled = end
            
        # convert to torch
        coord_t = torch.from_numpy( coord_np ).to(torch.device("cpu"))
        input_t = torch.from_numpy( input_np ).to(torch.device("cpu"))

        tloadbatchdata += time.time()-starttime

        starttime = time.time()
        # run through the model
        with torch.set_grad_enabled(False):
            out_t = model(coord_t, input_t, batchsize)

        out_np = out_t.detach().cpu().numpy()
        trunmodel += time.time()-starttime

        # packoutput back into bson
        starttime = time.time()

        # loop over batch
        nfilled = 0

        for ib in range(batchsize):

            img_idx  = ibatch*batchsize + ib
            if img_idx>=len(sparsedata_v):
                continue

            rsei = rse_v[img_idx]
            meta = sparsedata_v[img_idx].meta_v().front()

            outmeta_v = std.vector("larcv::ImageMeta")()
            outmeta_v.push_back( meta )

            start  = nfilled
            end    = nfilled+npts_v[ib]

            sparse_np = np.zeros( (npts_v[ib],3), dtype=np.float32 )
            sparse_np[:,0:2] = coord_np[start:end,0:2]
            sparse_np[:,2]   = out_np[start:end,0]

            # make the sparseimage object
            sparseimg = larcv.sparseimg_from_ndarray( sparse_np,
                                                      outmeta_v,
                                                      larcv.msg.kNORMAL )

            # convert to bson string
            print("[{}] {}".format(img_idx,rsei))
            bson = larcv.json.as_bson_pybytes( sparseimg, rsei[0], rsei[1], rsei[2], rsei[3] )

            # store in output list
            bson_results_v.append(bson)
            
        tpackoutput = time.time()-starttime

    return bson_results_v

if __name__ == "__main__":

    """ for testing """
    
    print("Test Inference Sparse-Infill")

    supera_file = sys.argv[1]
    io = larcv.IOManager(larcv.IOManager.kREAD,"supera",larcv.IOManager.kTickBackward)
    io.add_in_file( supera_file )
    io.initialize()

    weights = [ "sparseinfill_uplane_test.tar", 
                "sparseinfill_vplane_test.tar",
                "sparseinfill_yplane_test.tar" ]

    # splitter
    cfg = "../infill_split.cfg"

    pset = larcv.CreatePSetFromFile( cfg, "UBSplitDetector" )
    print(pset.dump())

    ubsplit = ublarcvapp.UBSplitDetector()
    ubsplit.configure(pset);
    ubsplit.initialize();

    ubbadch = ublarcvapp.EmptyChannelAlgo()

    nentries = io.get_n_entries()

    for ientry in range(nentries):
        io.read_entry(ientry)

        # Event Image
        ev_img = io.get_data( larcv.kProductImage2D, "wire" )
        img_v  = ev_img.Image2DArray()
        img_np_v = [ larcv.as_ndarray(img_v.at(p)) for p in range(3) ]

        # ChStatus
        ev_chstatus = io.get_data( larcv.kProductChStatus, "wire" )
        label_v = ubbadch.makeBadChImage( 4, 3, 2400, 
                                          int(img_v.at(0).meta().max_y()-img_v.at(0).meta().min_y()),
                                          3456,
                                          6, 1, ev_chstatus )

        cropadc_v = std.vector("larcv::Image2D")()
        adc_roi_v = std.vector("larcv:ROI")()
        ubsplit.process( img_v, cropadc_v, adc_roi_v )

        croplabel_v = std.vector("larcv::Image2D")()
        label_roi_v = std.vector("larcv:ROI")()

        ubsplit.process( label_v, croplabel_v, label_roi_v )


        print("Splits: ADC={} Labels={}".format(cropadc_v.size(),croplabel_v.size()))

        nsets = cropadc_v.size()/3

        # make list of sparse images
        sparse_v = {0:[],1:[],2:[]}
        thresholds = std.vector("float")(1,10.0)
        for iset in range(nsets):
            print("Set [{}]".format(iset))
            for p in [0,1,2]:
                crop  = cropadc_v.at( 3*iset+p )
                label = croplabel_v.at( 3*iset+p )
                sparseimg = larcv.SparseImage( crop, label, thresholds )
        
                bson = larcv.json.as_bson_pybytes( sparseimg, 
                                                   io.event_id().run(),
                                                   io.event_id().subrun(),
                                                   io.event_id().event(), iset )
                sparse_v[p].append(bson)
            if iset>=5:
                break
                
        print("Run nets")
        for p in [0,1,2]:
            out_v = forwardpass( sparse_v[p], weights[p] )
            print("plane {} returned with {} outputs".format(p,len(out_v)))
                  
        break
    
