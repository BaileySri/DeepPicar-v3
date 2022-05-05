#!/usr/bin/env python
##########################################################
# input config 
##########################################################
img_width = 200
img_height = 66
img_channels = 3

##########################################################
# model selection
#   "model_large"   <-- nvidia dave-2 model
##########################################################
model_name = "model_large"
model_file = "models/{}-{}x{}x{}".format(model_name[6:], img_width, img_height, img_channels)

##########################################################
# recording config 
##########################################################
rec_vid_file="out-video.avi"
rec_csv_file="out-key.csv"
