#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import subprocess

xaxis = []
for i in range(1,11):
	xaxis += [i]

proc = subprocess.Popen(["../../build/bin/./nanoflann_testReal"], stdout=subprocess.PIPE, shell=True)
(out, err) = proc.communicate()
nanoflannList = out.split()
numSamples = len(nanoflannList)/2
nanoflannBuildTime = []
nanoflannQueryTime = []
for it, time in enumerate(nanoflannList):
	if(it<numSamples):
		nanoflannBuildTime += [float(time)]
	else:
		nanoflannQueryTime += [float(time)]

proc = subprocess.Popen(["../../build/bin/./flann_testReal"], stdout=subprocess.PIPE, shell=True)
(out, err) = proc.communicate()
flannList = out.split()
numSamples = len(flannList)/2
flannBuildTime = []
flannQueryTime = []
for it, time in enumerate(flannList):
	if(it<numSamples):
		flannBuildTime += [float(time)]
	else:
		flannQueryTime += [float(time)]

proc = subprocess.Popen(["../../build/bin/./fastann_testReal"], stdout=subprocess.PIPE, shell=True)
(out, err) = proc.communicate()
fastannList = out.split()
numSamples = len(fastannList)/2
fastannBuildTime = []
fastannQueryTime = []
for it, time in enumerate(fastannList):
	if(it<numSamples):
		fastannBuildTime += [float(time)]
	else:
		fastannQueryTime += [float(time)]

maxBuildTime=max(max(nanoflannBuildTime), max(flannBuildTime), max(fastannBuildTime))+0.00001
maxQueryTime=max(max(nanoflannQueryTime), max(flannQueryTime), max(fastannQueryTime))+0.0000001

proc = subprocess.Popen(["../../build/bin/./libkdtree_testReal"], stdout=subprocess.PIPE, shell=True)
(out, err) = proc.communicate()
libkdtreeList = out.split()
numSamples = len(libkdtreeList)/2
libkdtreeBuildTime = []
libkdtreeQueryTime = []
for it, time in enumerate(libkdtreeList):
	if(it<numSamples):
		libkdtreeBuildTime += [min(float(time), maxBuildTime)]
	else:
		libkdtreeQueryTime += [min(float(time), maxQueryTime)]

fig, ax = plt.subplots()
ax.plot(xaxis, nanoflannBuildTime, 'r', label='nanoflann')
ax.plot(xaxis, nanoflannBuildTime, 'ro')
ax.plot(xaxis, flannBuildTime, 'b', label='flann')
ax.plot(xaxis, flannBuildTime, 'bo')
ax.plot(xaxis, fastannBuildTime, 'g', label='fastann')
ax.plot(xaxis, fastannBuildTime, 'go')
ax.plot(xaxis, libkdtreeBuildTime, 'k', label='libkdtree (values clipped)')
ax.plot(xaxis, libkdtreeBuildTime, 'ko')
plt.axis([0, 10.5, -0.00001, 0.00011])
plt.xlabel('Relative size of point cloud', fontsize=25)
plt.ylabel('Time (sec)', fontsize=25)
plt.title('kd-tree build time', fontsize=25)
# Now add the legend with some customizations.
legend = ax.legend(loc='upper center', shadow=True)

# The frame is matplotlib.patches.Rectangle instance surrounding the legend.
frame = legend.get_frame()
frame.set_facecolor('0.90')

# Set the fontsize
for label in legend.get_texts():
    label.set_fontsize('large')

for label in legend.get_lines():
    label.set_linewidth(1.5)  # the legend line width
plt.show()


fig, ax = plt.subplots()
ax.plot(xaxis, nanoflannQueryTime, 'r', label='nanoflann')
ax.plot(xaxis, nanoflannQueryTime, 'ro')
ax.plot(xaxis, flannQueryTime, 'b', label='flann')
ax.plot(xaxis, flannQueryTime, 'bo')
ax.plot(xaxis, fastannQueryTime, 'g', label='fastann')
ax.plot(xaxis, fastannQueryTime, 'go')
ax.plot(xaxis, libkdtreeQueryTime, 'k', label='libkdtree (values clipped)')
ax.plot(xaxis, libkdtreeQueryTime, 'ko')
plt.axis([0, 10.5, -0.0000001, 0.0000035])
plt.xlabel('Relative size of point cloud', fontsize=25)
plt.ylabel('Time (sec)', fontsize=25)
plt.title('One 3d query time', fontsize=25)
# Now add the legend with some customizations.
legend = ax.legend(loc='upper center', shadow=True)

# The frame is matplotlib.patches.Rectangle instance surrounding the legend.
frame = legend.get_frame()
frame.set_facecolor('0.90')

# Set the fontsize
for label in legend.get_texts():
    label.set_fontsize('large')

for label in legend.get_lines():
    label.set_linewidth(1.5)  # the legend line width
plt.show()
