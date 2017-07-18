#!/usr/bin/python
import math
import matplotlib.pyplot as plt
import subprocess

def cal(MatrixTime, col):
	meanVal = 0.0
	stdVal  = 0.0
	numRepetitions = len(MatrixTime)
	for i in range(numRepetitions):
		meanVal = meanVal + MatrixTime[i][col][0]
	meanVal /= numRepetitions
	for i in range(numRepetitions):
		stdVal = stdVal + (MatrixTime[i][col][0]-meanVal)*(MatrixTime[i][col][0]-meanVal)
	stdVal /= numRepetitions
	stdVal = stdVal**0.5
	return meanVal, stdVal


def plotTime(execPath, numRepetitions, numDivisions):
	BuildTime = [[0.0 for x in range(numDivisions)] for y in range(numRepetitions)] 
	QueryTime = [[0.0 for x in range(numDivisions)] for y in range(numRepetitions)] 
	for processCount in range(numRepetitions):
		proc = subprocess.Popen([execPath], stdout=subprocess.PIPE, shell=True)
		(out, err) = proc.communicate()
		List = out.split()
		for it, time in enumerate(List):
			if(it<numDivisions):
				BuildTime[processCount][it] = [float(time)*1000]
			else:
				QueryTime[processCount][it-10] = [float(time)*1000]
	BuildTimeFinal = []
	BuildTimeError = []
	QueryTimeFinal = []
	QueryTimeError = []
	for col in range(numDivisions):
		meanVal, stdVal = cal(BuildTime, col)
		BuildTimeFinal += [meanVal]
		BuildTimeError += [stdVal]
		meanVal, stdVal = cal(QueryTime, col)
		QueryTimeFinal += [meanVal]
		QueryTimeError += [stdVal]
	return BuildTimeFinal, BuildTimeError, QueryTimeFinal, QueryTimeError

numRepetitions = 3
numDivisions = 10
xaxis = []
for i in range(1,numDivisions+1):
	xaxis += [i]


fig, ax = plt.subplots()
nanoflannBuildTimeFinal, nanoflannBuildTimeError, nanoflannQueryTimeFinal, nanoflannQueryTimeError = plotTime('../../build/bin/./nanoflann_testRandom', numRepetitions, numDivisions)
flannBuildTimeFinal, flannBuildTimeError, flannQueryTimeFinal, flannQueryTimeError = plotTime('../../build/bin/./flann_testRandom', numRepetitions, numDivisions)
fastannBuildTimeFinal, fastannBuildTimeError, fastannQueryTimeFinal, fastannQueryTimeError = plotTime('../../build/bin/./fastann_testRandom', numRepetitions, numDivisions)
plt.plot(xaxis, nanoflannBuildTimeFinal, 'r', label='nanoflann', linewidth=3.0)
plt.errorbar(xaxis, nanoflannBuildTimeFinal, color='k', yerr=nanoflannBuildTimeError, fmt='o')
plt.plot(xaxis, flannBuildTimeFinal, 'g', label='flann', linewidth=3.0)
plt.errorbar(xaxis, flannBuildTimeFinal, color='k', yerr=flannBuildTimeError, fmt='o')
plt.plot(xaxis, fastannBuildTimeFinal, 'b', label='fastann', linewidth=3.0)
plt.errorbar(xaxis, fastannBuildTimeFinal, color='k', yerr=fastannBuildTimeError, fmt='o')
plt.xlabel('Relative size of point cloud', fontsize=25)
plt.ylabel('Time (ms)', fontsize=25)
plt.title('kd-tree build time', fontsize=25)
# Now add the legend with some customizations.
legend = ax.legend(loc='upper left', shadow=True)
# The frame is matplotlib.patches.Rectangle instance surrounding the legend.
frame = legend.get_frame()
frame.set_facecolor('0.90')
# Set the fontsize
for label in legend.get_texts():
    label.set_fontsize('large')
for label in legend.get_lines():
    label.set_linewidth(10)  # the legend line width
plt.show()

fig, ax = plt.subplots()
plt.plot(xaxis, nanoflannQueryTimeFinal, 'r', label='nanoflann', linewidth=3.0)
plt.errorbar(xaxis, nanoflannQueryTimeFinal, color='k', yerr=nanoflannQueryTimeError, fmt='o')
plt.plot(xaxis, flannQueryTimeFinal, 'g', label='flann', linewidth=3.0)
plt.errorbar(xaxis, flannQueryTimeFinal, color='k', yerr=flannQueryTimeError, fmt='o')
plt.plot(xaxis, fastannQueryTimeFinal, 'b', label='fastann', linewidth=3.0)
plt.errorbar(xaxis, fastannQueryTimeFinal, color='k', yerr=fastannQueryTimeError, fmt='o')
plt.xlabel('Relative size of point cloud', fontsize=25)
plt.ylabel('Time (ms)', fontsize=25)
plt.title('One 3d query time', fontsize=25)
# Now add the legend with some customizations.
legend = ax.legend(loc='upper left', shadow=True)
# The frame is matplotlib.patches.Rectangle instance surrounding the legend.
frame = legend.get_frame()
frame.set_facecolor('0.90')
# Set the fontsize
for label in legend.get_texts():
    label.set_fontsize('large')
for label in legend.get_lines():
    label.set_linewidth(10)  # the legend line width
plt.show()