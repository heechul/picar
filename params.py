
#!/usr/bin/env python 
from __future__ import division

import os
from collections import OrderedDict

batch_size = 100

VER="-donkey-T789101112-model2-morning"
save_dir = os.path.abspath('models' + VER)

training_steps = 2000
img_height = 66
img_width = 200
img_channels = 3
write_summary = True
use_category_normal = True # False # if ture, center/curve images are equally selected.

# change this to the directory that contains the source videos
data_dir = os.path.abspath('epochs')
out_dir = os.path.abspath('output' + VER)

# data_dir = os.path.abspath('epochs')
# out_dir = os.path.abspath('.output')

shuffle_training = True

assert os.path.isdir(data_dir)
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

epochs = OrderedDict()
# epochs['train'] = [7,8,9,10]
# epochs['val'] = [11,12]

epochs['all'] = [7,8,9,10,11,12]
train_pct = 0.8
