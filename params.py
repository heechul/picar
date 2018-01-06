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
use_category_normal = True # if ture, center/curve images are equally selected.
shuffle_training = True

# change this to the directory that contains the source videos
data_dir = os.path.abspath('epochs-conv')
save_dir = os.path.abspath('models-5conv_3fc-home')
out_dir = os.path.abspath('output-5conv_3fc-home')

assert os.path.isdir(data_dir)
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

epochs = OrderedDict()
epochs['train'] = [100, 101, 103, 104] # range(30, 50)
epochs['val'] = [102, 105] # range(50, 60)

