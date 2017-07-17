#!/usr/bin/env python 
from __future__ import division

import os
from collections import OrderedDict

batch_size = 100
# save_dir = os.path.abspath('models')
save_dir = os.path.abspath('../picar/models-dagger')
training_steps = 2000
img_height = 66
img_width = 200
img_channels = 3
write_summary = True

# change this to the directory that contains the source videos
data_dir = os.path.abspath('../picar/epochs')
out_dir = os.path.abspath('../picar/output-dagger')
# data_dir = os.path.abspath('epochs')
# out_dir = os.path.abspath('.output')

shuffle_training = True

assert os.path.isdir(data_dir)
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

epochs = OrderedDict()

epochs['train'] = [100,101,102,103,104,105,106,107,108,109,110] 
epochs['val'] = [100] 

# epochs['train'] = [6, 7, 8, 14, 15, 16, 18, 19, 20, 22] 
# epochs['val'] = [13, 17, 21] 
# epochs['train'] = [18,19,20] # range(0,5)
# epochs['val'] = [21] # range(0,5)
# epochs['train'] = [3, 4, 5, 6, 8]
# epochs['val'] = [1, 2, 7, 9, 10]

