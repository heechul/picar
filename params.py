#!/usr/bin/env python 
from __future__ import division

import os
from collections import OrderedDict

batch_size = 100
training_steps = 2000
img_height = 66
img_width = 200
img_channels = 3

write_summary = True
shuffle_training = True
use_category_normal = True # if ture, center/curve images are equally selected.
use_picar_mini = True

# change this to the directory that contains the source videos
# "-5conv_3fc_noreuse" "-5conv_3fc" "-3conv_1pool" "-5conv_3fc_noreuse_nocn"
model="model-5conv_3fc"
# model_load_file="model-5conv_3fc_noreuse.ckpt"  # <-- solved ittc building maze. 
model_load_file="model-5conv_3fc-home_night.ckpt"
model_save_file=model_load_file
save_dir = os.path.abspath('models')
data_dir = os.path.abspath('epochs-conv')
out_dir = os.path.abspath('output')

if not os.path.isdir(data_dir):
    os.makedirs(data_dir)
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

epochs = OrderedDict()
epochs['train'] = range(30, 50) + range(60, 68) 
epochs['val'] = range(50, 60) + range(68, 70)

# data description
# range(30, 60)        # ittc building. michael. resynchronized
# range(60, 70)        # ittc building. yun
# range(100, 106)      # home. yun

