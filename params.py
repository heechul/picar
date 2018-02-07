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
use_category_normal = False # if ture, center/curve images are equally selected.

# change this to the directory that contains the source videos
save_dir = os.path.abspath('MiniModelNew')
save_dir2 = os.path.abspath('MiniModelNew2')
save_dir3 = os.path.abspath('MiniModelNew3')
save_dir4 = os.path.abspath('MiniModelNew4')
data_dir = os.path.abspath('epochs')
out_dir = os.path.abspath('output')

shuffle_training = True

assert os.path.isdir(data_dir)
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

epochs = OrderedDict()
epochs['train'] = [] # epoch_ids for training data: e.g., "epochs['train'] = [1, 2, 3]" 
epochs['val'] = [] # epoch_ids for validation data: e.g., "epochs['val'] = [4]" 
