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
use_model_load = False # initialize weights from reading the existing 
use_picar_mini = True

# change this to the directory that contains the source videos
# "-5conv_3fc_noreuse" "-5conv_3fc" "-3conv_1pool" "-5conv_3fc_noreuse_nocn"
VER="-3conv_1pool_nocn"
model_name = 'model' + VER + '.ckpt'
data_dir = os.path.abspath('epochs-conv')
save_dir = os.path.abspath('models')
out_dir = os.path.abspath('output')

assert os.path.isdir(data_dir)
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

epochs = OrderedDict()
epochs['train'] = range(30, 50) + range(60, 68) 
epochs['val'] = range(50, 60) + range(68, 70)

# data description
# range(30, 60)        # ittc building. michael. resynchronized
# range(60, 70)        # ittc building. yun
# range(100, 106)      # home. yun

