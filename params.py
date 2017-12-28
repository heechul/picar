#!/usr/bin/env python 
from __future__ import division

import os
from collections import OrderedDict

batch_size = 255

VER="-donkey-T78910_V1112"
save_dir = os.path.abspath('../picar/models' + VER)

training_steps = 2000
img_height = 66
img_width = 200
img_channels = 3
write_summary = True
use_category_normal = False # if ture, center/curve images are equally selected.

# change this to the directory that contains the source videos
data_dir = os.path.abspath('../picar/epochs')
out_dir = os.path.abspath('../picar/output' + VER)

# data_dir = os.path.abspath('epochs')
# out_dir = os.path.abspath('.output')

shuffle_training = True

assert os.path.isdir(data_dir)
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

epochs = OrderedDict()

epochs['train'] = [7,8,9,10]
epochs['val'] = [11,12]

