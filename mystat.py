import os
import time
import numpy as np
from PIL import Image
import glob

# stats
class DeadlineStat:
    def __init__(self, deadline=0):
        self.deadline = deadline
        self.deadline_miss_cnt = 0
        self.exectime_min = 0 # min exec. time
        self.exectime_max = 0 # max exec. time
        self.exectime_sum = 0 # sum exec. times
        self.exectime_cnt = 0 # cnt exec. times

    def update(self, exec_time):
        self.exectime_cnt += 1
        self.exectime_sum += exec_time
        self.exectime_min = min(self.exectime_min, exec_time)
        self.exectime_max = max(self.exectime_max, exec_time)
        if exec_time > self.deadline:
            self.deadline_miss_cnt += 1
            
    def print_stat(self):
        print "Print statistics"
        print "================"
        print "min/avg/max (ms): {:.2f} {:.2f} {:.2f}".format(
            self.exectime_min * 1000,
            self.exectime_sum/self.exectime_cnt * 1000,
            self.exectime_max * 1000)
        print "deadline_miss_cnt: {}".format(self.deadline_miss_cnt)
        

        

