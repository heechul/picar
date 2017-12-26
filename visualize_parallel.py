#!/usr/bin/env python
from __future__ import division

import subprocess as sp
import os
import io
import sys
import re
from copy import deepcopy
import psycopg2
import psycopg2.extras
import subprocess
from operator import itemgetter
from collections import OrderedDict, Counter
import cv2
from fractions import Fraction
import csv
import errno
import time
import shutil
import numpy as np
import datetime
import matplotlib.pyplot as plt
import PIL
from pprint import pprint
import multiprocessing
import string
import copy
import json
import random
import params
import math
from joblib import Parallel, delayed 
import multiprocessing
import local_common as cm

machine_steering = []
human_steering = []
cam_images = []

wimg = cm.imread(os.path.abspath("images/wheel-tesla-image-150.png"), cv2.IMREAD_UNCHANGED)
timg_green_agree = cm.imread(os.path.abspath("images/text-green-agree.png"), cv2.IMREAD_UNCHANGED)
timg_ground_truth = cm.imread(os.path.abspath("images/text-ground-truth.png"), cv2.IMREAD_UNCHANGED)
timg_learned_control = cm.imread(os.path.abspath("images/text-learned-control.png"), cv2.IMREAD_UNCHANGED)
timg_red_disagree = cm.imread(os.path.abspath("images/text-red-disagree.png"), cv2.IMREAD_UNCHANGED)
timg_tesla_control_autopilot = cm.imread(os.path.abspath("images/text-tesla-control-autopilot.png"), cv2.IMREAD_UNCHANGED)
timg_tesla_control_human = cm.imread(os.path.abspath("images/text-tesla-control-human.png"), cv2.IMREAD_UNCHANGED)
# timg_ = cm.imread(os.path.abspath("images/text-.png"), cv2.IMREAD_UNCHANGED)

def get_human_steering(epoch_id):
    epoch_dir = params.data_dir
    assert os.path.isdir(epoch_dir)
    steering_path = cm.jn(epoch_dir, 'out-key-{}.csv'.format(epoch_id))
    # steering_path = cm.jn(epoch_dir, 'epoch{:0>2}_steering.csv'.format(epoch_id))
    assert os.path.isfile(steering_path)
    
    rows = cm.fetch_csv_data(steering_path)
    human_steering = [row['wheel'] for row in rows]
    return human_steering # <-- this is the original

def process_one_frame (f_cur):
    rimg = cam_images[f_cur]
    
    if dash_exists:
        dret, dimg = dash_cap.read()
        assert dret
    else:
        dimg = rimg.copy()
        dimg[:] = (0, 0, 0)
        
    ry0, rh = 80, 500
    dimg = dimg[100:, :930]
    dimg = cm.cv2_resize_by_height(dimg, h-rh)
    
    fimg = rimg.copy()
    fimg[:] = (0, 0, 0)
    
    # print "DBG:", rh, ry0, ry0+rh
        
    fimg[:rh] = rimg[ry0:ry0+rh] # DBG
    dh, dw = dimg.shape[:2]
    fimg[rh:,:dw] = dimg[:]
    
    ########################## plot ##########################
    plot_size = (500, dh)
    win_before, win_after = 150, 150
    
    xx, hh, mm = [], [], []
    for f_rel in xrange(-win_before, win_after+1):
        f_abs = f_cur + f_rel
        if f_abs < 0 or f_abs >= len(machine_steering):
            continue
        xx.append(f_rel/30)
        hh.append(human_steering[f_abs])
        mm.append(machine_steering[f_abs])

    fig = plt.figure()
    axis = fig.add_subplot(1, 1, 1)

    steering_range = max(abs(steering_min), abs(steering_max))
    #ylim = [steering_min, steering_max]
    ylim = [-steering_range, steering_range]
    # ylim[0] = min(np.min(hh), np.min(mm))
    # ylim[1] = max(np.max(hh), np.max(mm))
    
    axis.set_xlabel('Current Time (secs)')
    axis.set_ylabel('Steering Angle')
    axis.axvline(x=0, color='k', ls='dashed')
    axis.plot(xx, hh)
    axis.plot(xx, mm)
    axis.set_xlim([-win_before/30, win_after/30])
    axis.set_ylim(ylim)
    #axis.set_ylabel(y_label, fontsize=18)
    axis.label_outer()
    #axes.append(axis)
    
    buf = io.BytesIO()
    # http://stackoverflow.com/a/4306340/627517
    sx, sy = plot_size
    sx, sy = round(sx / 100, 1), round(sy / 100, 1)
    
    fig.set_size_inches(sx, sy)
    fig.tight_layout()
    fig.savefig(buf, format="png", dpi=100)
    buf.seek(0)
    buf_img = PIL.Image.open(buf)
    pimg = np.asarray(buf_img)
    plt.close(fig)
    
    pimg = cv2.resize(pimg, plot_size)
    pimg = pimg[:,:,:3]
    
    ph, pw = pimg.shape[:2]
    pimg = 255 - pimg
    
    fimg[rh:,-pw:] = pimg[:]
    
    ####################### human steering wheels ######################
    # wimg = cm.imread(os.path.abspath("images/wheel-tesla-image-150.png"), cv2.IMREAD_UNCHANGED)
    
    human_wimg = cm.rotate_image(wimg, -human_steering[f_cur])
    wh, ww = human_wimg.shape[:2]
    fimg = cm.overlay_image(fimg, human_wimg, y_offset = rh+50, x_offset = dw+60)
    
    ####################### machine steering wheels ######################
    disagreement = abs(machine_steering[f_cur] - human_steering[f_cur])
    machine_wimg = cm.rotate_image(wimg, -machine_steering[f_cur])
    red_machine_wimg = machine_wimg.copy()
    green_machine_wimg = machine_wimg.copy()
    red_machine_wimg[:,:,2] = 255
    green_machine_wimg[:,:,1] = 255
    #r = disagreement / (steering_max - steering_min)
    max_disagreement = 15
    r = min(1., disagreement / max_disagreement)
    g = 1 - r
    assert r >= 0
    assert g <= 1
    machine_wimg = cv2.addWeighted(red_machine_wimg, r, green_machine_wimg, g, 0)
    wh, ww = machine_wimg.shape[:2]
    fimg = cm.overlay_image(fimg, machine_wimg, y_offset = rh+50, x_offset = dw+260)
    
    ####################### text ######################
    # timg_green_agree = cm.imread(os.path.abspath("images/text-green-agree.png"), cv2.IMREAD_UNCHANGED)
    # timg_ground_truth = cm.imread(os.path.abspath("images/text-ground-truth.png"), cv2.IMREAD_UNCHANGED)
    # timg_learned_control = cm.imread(os.path.abspath("images/text-learned-control.png"), cv2.IMREAD_UNCHANGED)
    # timg_red_disagree = cm.imread(os.path.abspath("images/text-red-disagree.png"), cv2.IMREAD_UNCHANGED)
    # timg_tesla_control_autopilot = cm.imread(os.path.abspath("images/text-tesla-control-autopilot.png"), cv2.IMREAD_UNCHANGED)
    # timg_tesla_control_human = cm.imread(os.path.abspath("images/text-tesla-control-human.png"), cv2.IMREAD_UNCHANGED)
    # # timg_ = cm.imread(os.path.abspath("images/text-.png"), cv2.IMREAD_UNCHANGED)
    
    fimg = cm.overlay_image(fimg, timg_tesla_control_human, y_offset = rh+8, x_offset = dw+83)
    fimg = cm.overlay_image(fimg, timg_learned_control, y_offset = rh+8, x_offset = dw+256)
    fimg = cm.overlay_image(fimg, timg_ground_truth, y_offset = rh+205, x_offset = dw+90)
    fimg = cm.overlay_image(fimg, timg_red_disagree, y_offset = rh+205, x_offset = dw+230)
    fimg = cm.overlay_image(fimg, timg_green_agree, y_offset = rh+205, x_offset = dw+345)
    
    return fimg

def visualize(epoch_id, machine_steering_local, out_dir, perform_smoothing=False,
              verbose=False, verbose_progress_step = 100, frame_count_limit = None):
    global cam_images
    global steering_min, steering_max
    global dash_exists
    global w,h
    global human_steering, machine_steering
    machine_steering = []
    for i in machine_steering_local:
        machine_steering.append(i)

    epoch_dir = params.data_dir
    human_steering = get_human_steering(epoch_id)
    assert len(human_steering) == len(machine_steering)

    # testing: artificially magnify steering to test steering wheel visualization
    # human_steering = list(np.array(human_steering) * 10)
    # machine_steering = list(np.array(machine_steering) * 10)
    # testing: artificially alter machine steering to test that the disagreement coloring is working
    # delta = 0
    # for i in xrange(len(machine_steering)):
    #     delta += random.uniform(-1, 1)
    #     machine_steering[i] += delta
    if perform_smoothing:
        machine_steering = list(cm.smooth(np.array(machine_steering)))
        #human_steering = list(cm.smooth(np.array(human_steering)))

    steering_min = min(np.min(human_steering), np.min(machine_steering))
    steering_max = max(np.max(human_steering), np.max(machine_steering))

    assert os.path.isdir(epoch_dir)

    front_vid_path = cm.jn(epoch_dir, 'out-video-{}-scaled.avi'.format(epoch_id))
    assert os.path.isfile(front_vid_path)
    
    dash_vid_path = cm.jn(epoch_dir, 'epoch{:0>2}_dash.mkv'.format(epoch_id))
    dash_exists = os.path.isfile(dash_vid_path)

    front_cap = cv2.VideoCapture(front_vid_path)
    dash_cap = cv2.VideoCapture(dash_vid_path) if dash_exists else None
    
    assert os.path.isdir(out_dir)
    vid_size = cm.video_resolution_to_size('720p', width_first=True)
    out_path = cm.jn(out_dir, 'out-video-{}-human_machine.mkv'.format(epoch_id))
    # fourcc = cv2.VideoWriter_fourcc(*'XVID' ) # opencv 3.x
    fourcc = cv2.cv.CV_FOURCC(*'XVID' ) # opencv 2.x
    vw = cv2.VideoWriter(out_path, fourcc, 30, vid_size)
    w, h = vid_size

    #input lists
    cam_images = []
    for i in xrange (len (machine_steering)):
        rret, rimg = front_cap.read()
        assert rret
	cam_images.append(rimg)
        
    #output image lists
    num_cores = multiprocessing.cpu_count()
    out_images = Parallel (n_jobs = num_cores) (delayed (process_one_frame) (i) for i in range (len (machine_steering)))

    for fimg in out_images:
        vw.write(fimg)

    front_cap.release()
    if dash_exists:
        dash_cap.release()
    vw.release()

    machine_steering = []
    cam_images = []
    cm.mkv_to_mp4(out_path, remove_mkv=True)
    

        
if __name__ == '__main__':
    epoch_id = 7
    machine_steering = get_human_steering(epoch_id)

    # frame_count_limit = None
    # frame_count_limit = 30 * 5
    # frame_count_limit = 1
    visualize(epoch_id, machine_steering, params.out_dir,
              verbose=True, frame_count_limit=150)
