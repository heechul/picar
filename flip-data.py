import cv2
import local_common as cm
import os
import sys

import cfg

font = cv2.FONT_HERSHEY_SIMPLEX

if len(sys.argv) == 2:
    epoch_id = int(sys.argv[1])
    vid_path = 'epochs/out-video-{}.avi'.format(epoch_id)
    csv_path = 'epochs/out-key-{}.csv'.format(epoch_id)
    vid_flip_path = 'epochs/out-video-{}-flip.avi'.format(epoch_id)
    csv_flip_path = 'epochs/out-key-{}-flip.csv'.format(epoch_id)
else:
    raise "ERROR"

assert os.path.isfile(vid_path)
assert os.path.isfile(csv_path)

# input data files
frame_count = cm.frame_count(vid_path)
cap = cv2.VideoCapture(vid_path)
rows = cm.fetch_csv_data(csv_path)
assert frame_count == len(rows)

# output data files
# fourcc = cv2.cv.CV_FOURCC(*'XVID')
fourcc = cv2.VideoWriter_fourcc(*'XVID')

vidfile_flip = cv2.VideoWriter(vid_flip_path, fourcc, cfg.cam_fps,
                          (cfg.cam_width, cfg.cam_height))
keyfile_flip = open(csv_flip_path, 'w+')
keyfile_flip.write("ts_micro,frame,wheel\n")

# main loop
for i in range(frame_count):
    # image flip
    ret, img = cap.read()
    img_flip = cv2.flip(img, 1)
    vidfile_flip.write(img_flip)
    
    # steering data flip
    text = "{},{},{}\n".format(rows[i]['ts_micro'],
                             rows[i]['frame'],
                             -rows[i]['wheel'])  # change direction
    keyfile_flip.write(text)

print "Closing the files..."    
cap.release()
keyfile_flip.close()
vidfile_flip.release()
