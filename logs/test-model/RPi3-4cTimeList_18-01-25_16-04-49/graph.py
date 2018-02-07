from matplotlib import pyplot as plt
import numpy as np

list = open("4CTimeList.txt").readlines()

plt.plot(list,'k')
plt.xlabel('Frame Number', fontname="Times New Roman", fontsize=14)
plt.ylabel('Processing Time(ms)', fontname="Times New Roman", fontsize=14)
plt.axis([0, 1000, 0, 40])
plt.show()