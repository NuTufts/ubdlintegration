from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os,sys
import numpy as np
import torch
from larcv import larcv
from ctypes import c_int
larcv.json.load_jsonutils()

def forwardpass( dense_img_bson, weights_filepath, cfg_filepath ):

    # mask rcnn: need mask-rcnn.pytorch/lib in PYTHONPATH
    from utils.collections import AttrDict
    from core.config import make_default_config
    import core.config

    core.config.__C = make_default_config()
    core.config.cfg = core.config.__C

    from core.config import cfg, cfg_from_file, cfg_from_list, assert_and_infer_cfg

    cfg_from_file( cfg_filepath )
    cfg.TRAIN.DATASETS = ('particle_physics_train')
    cfg.MODEL.NUM_CLASSES = 7
    cfg.MODEL.LOAD_IMAGENET_PRETRAINED_WEIGHTS = False  # Don't need to load imagenet pretrained weights
    cfg.MODEL.DEVICE = "cpu"

    from core.test import im_detect_all
    import utils.net as net_utils
    import nn as maskrcnn_nn
    from modeling.model_builder import Generalized_RCNN

    assert_and_infer_cfg()
    model = Generalized_RCNN()

    locations = {}
    for x in range(10):
        locations["cuda:%d"%(x)] = "cpu"
    checkpoint = torch.load(weights_filepath, map_location=locations)
    net_utils.load_ckpt(model, checkpoint['model'])

    model = maskrcnn_nn.DataSingular(model, cpu_keywords=['im_info', 'roidb'],
                                     minibatch=True , device_id=[cfg.MODEL.DEVICE])

    model.eval()

    print("Loaded Model")
    c_run = c_int()
    c_subrun = c_int()
    c_event = c_int()
    c_id = c_int()
    img2d = larcv.json.image2d_from_pybytes(dense_img_bson,
                                            c_run, c_subrun, c_event, c_id )
    meta  = img2d.meta()
    print("meta: {}".format(meta.dump()))

    img_batch_np = larcv.as_ndarray( img2d ).reshape( (1,1,meta.cols(),meta.rows()) )
    print( type(img_batch_np) )

    height = img_batch_np.shape[2]
    width  = img_batch_np.shape[3]

    # New and Fast
    im = np.array([np.copy(img_batch_np[0][0]),np.copy(img_batch_np[0][0]),np.copy(img_batch_np[0][0])])
    im = np.moveaxis(np.moveaxis(im,0,2),0,1)

    assert im is not None
    thresh = 0.7
    print("Using a score threshold of 0.7 to cut boxes. Hard Coded")
    clustermasks_this_img = []
    cls_boxes, cls_segms, cls_keyps, round_boxes = im_detect_all(model, im, timers=None, use_polygon=False)
    np.set_printoptions(suppress=True)
    for cls in range(len(cls_boxes)):
        assert len(cls_boxes[cls]) == len(cls_segms[cls])
        assert len(cls_boxes[cls]) == len(round_boxes[cls])
        for roi in range(len(cls_boxes[cls])):
            if cls_boxes[cls][roi][4] > thresh:
                segm_coo = cls_segms[cls][roi].tocoo()
                non_zero_num = segm_coo.count_nonzero()
                segm_np = np.zeros((non_zero_num, 2), dtype=np.float32)
                counter = 0
                for i,j,v in zip(segm_coo.row, segm_coo.col, segm_coo.data):
                    segm_np[counter][0] = j
                    segm_np[counter][1] = i
                    counter = counter+1
                round_box = np.array(round_boxes[cls][roi], dtype=np.float32)
                round_box = np.append(round_box, np.array([cls], dtype=np.float32))

                clustermasks_this_img.append(larcv.as_clustermask(segm_np, round_box, meta, np.array([cls_boxes[cls][roi][4]], dtype=np.float32)))

                ### Checks to make sure the clustermasks being placed
                ### in the list have the appropriate values relative
                ### to what we send the pyutil
                # cmask = clustermasks_this_img[len(list_clustermasks)-1]
                # print()
                # print(round_box)
                # print("Segm shape", segm_np.shape)
                # print(segm_np[0][0] , segm_np[0][1])
                # print(segm_np[1][0] , segm_np[1][1])
                # print()
                # print(cmask.box.min_x(), cmask.box.min_y(), "   ", cmask.box.max_x(), cmask.box.max_y(), "    " , cmask.type)
                # print(cmask._box.at(0), cmask._box.at(1), "   ", cmask._box.at(2), cmask._box.at(3), "    " , cmask._box.at(4))
                # print("points_v len", len(cmask.points_v))
                # print(cmask.points_v.at(0).x, cmask.points_v.at(0).y)
                # print(cmask.points_v.at(1).x, cmask.points_v.at(1).y)
    

    cluster_mask_json_v = []
    for mask_idx in range(len(clustermasks_this_img)):
        mask = clustermasks_this_img[mask_idx]
        # print(mask.as_vector_box_no_convert()[0], mask.as_vector_box_no_convert()[1], mask.as_vector_box_no_convert()[2], mask.as_vector_box_no_convert()[3])
        meta  = mask.meta
        # print((meta.dump()))
        # print(type(mask))
        # out_img2d = larcv.as_image2d_meta( out_np.reshape((1008,3456)), meta )
        bson = larcv.json.as_pybytes( mask,
                                      c_run.value, 
                                      c_subrun.value, 
                                      c_event.value, 
                                      c_id.value )
        cluster_mask_json_v.append(bson)


    del cfg
    return cluster_mask_json_v



if __name__ == "__main__":
    
    """ for testing """
    
    print("Test Inference Mask-RCNN")

    supera_file = sys.argv[1]
    io = larcv.IOManager(larcv.IOManager.kREAD,"supera",larcv.IOManager.kTickBackward)
    io.add_in_file( supera_file )
    io.initialize()

    weights = [ os.environ["UBLARCVSERVER_BASEDIR"]+"/app/ubmrcnn/weights/ubmrcnn_mcc8_v1/mcc8_mrcnn_plane{}.pth".format(x) for x in range(3) ]
    configs = [ "mills_config_{}.yaml".format(x) for x in range(3) ]

    nentries = io.get_n_entries()

    for ientry in range(nentries):
        io.read_entry(ientry)

        ev_img = io.get_data( larcv.kProductImage2D, "wire" )
        img_v  = ev_img.Image2DArray()
        img_np_v = [ larcv.as_ndarray(img_v.at(p)) for p in range(3) ]
        
        for p in [0,1,2]:
            print("plane {}".format(p))
            bson = larcv.json.as_pybytes( img_v.at(p),
                                          io.event_id().run(),
                                          io.event_id().subrun(),
                                          io.event_id().event(), 0 )

            out = forwardpass( bson, weights[p], configs[p] )
            print("plane {} returned with {} outputs".format(p,len(out)))
                  
        break
