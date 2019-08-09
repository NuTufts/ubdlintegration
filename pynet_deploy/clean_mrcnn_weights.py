from __future__ import print_function
import os,sys
import torch

weight_dir = ["."]

for p in [0,1,2]:
    f = open("mcc8_mrcnn_plane%d_weightsonly.dat"%(p),'w')
    data = torch.load("mcc8_mrcnn_plane%d.pth"%(p),map_location={'cuda:0':'cpu'})
    data_dict = {"model":data['model']}
    torch.save(data_dict,f)
    print("made: {}".format("mcc8_mrcnn_plane%d_weightsonly.dat"%(p)))
    f.close()
