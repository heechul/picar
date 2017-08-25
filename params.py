#!/usr/bin/env python 
from __future__ import division

import os
from collections import OrderedDict

batch_size = 100
# save_dir = os.path.abspath('models')
save_dir = os.path.abspath('../picar/models-v21-lr-dagger')
training_steps = 2000
img_height = 66
img_width = 200
img_channels = 3
write_summary = True
use_category_normal = False # if ture, center/curve images are equally selected.

# change this to the directory that contains the source videos
data_dir = os.path.abspath('../picar/epochs')
out_dir = os.path.abspath('../picar/output-v21-lr-dagger')

# data_dir = os.path.abspath('epochs')
# out_dir = os.path.abspath('.output')

shuffle_training = True

assert os.path.isdir(data_dir)
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

epochs = OrderedDict()
# 8/23 picar-v2.0, 50ms fix

epochs['train'] = [401, 402, 403, 404, 409, 410, 411, 412]
epochs['train'] += [406, 407, 408] # dagger
epochs['val'] = [405, 413]

# epochs['train'] += [414, 415, 416, 417, 419, 420, 421, 422]
# epochs['val'] += [418, 423]

# 8/21 picar-v2.0
# left = 302 - 306
# right = 307 - 311
# epochs['train'] = [302, 303, 304, 305, 307, 308, 309, 310]
# epochs['val'] = [306, 311] 

# center following.
# left  = 120-135 (18)
# right = 117-119, 136 - 144, 100-101 (14)
# epochs['train'] = [100, 101, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144]
# epochs['val'] =   [145, 146]


# # right only
# epochs['train'] = [100, 101, 110, 111, 117, 118]
# epochs['val'] =   [112, 119]

# right & left (120-124)
# epochs['train'] = [100, 101, 110, 111, 117, 118, 120, 121, 122, 123]
# epochs['val'] =   [112, 119, 124]

# epochs['train'] = [100, 102, 104, 106, 108, 110, 112, 114, 103, 105, 107, 109, 113, 115] 
# epochs['val'] =   [101, 111]

# step   10 of 2000, train loss 0.177199780941,   val loss 0.183928459883
# step 1000 of 2000, train loss 0.00693200016394, val loss 0.0959444642067
# step 2000 of 2000, train loss 0.00296380673535, val loss 0.0754969492555


# epochs['train'] = [100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115] 
# epochs['val'] = [100] 

# epochs['train'] = [6, 7, 8, 14, 15, 16, 18, 19, 20, 22] 
# epochs['val'] = [13, 17, 21] 
# epochs['train'] = [18,19,20] # range(0,5)
# epochs['val'] = [21] # range(0,5)
# epochs['train'] = [3, 4, 5, 6, 8]
# epochs['val'] = [1, 2, 7, 9, 10]

