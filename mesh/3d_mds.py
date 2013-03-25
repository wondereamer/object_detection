#!/usr/bin/python
"""
=========================
Multi-dimensional scaling
=========================

An illustration of the metric and non-metric MDS on generated noisy data.

The reconstructed points using the metric MDS and non metric MDS are slightly
shifted to avoid overlapping.
"""

# Author: Nelle Varoquaux <nelle.varoquaux@gmail.com>
# Licence: BSD

print __doc__
import numpy as np

from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
import numpy as np
#from matplotlib.collections import LineCollection

from sklearn import manifold
from sklearn.metrics import euclidean_distances
import fileinput                         

xList = []
yList = []
zList = []
for line in fileinput.input(inplace = False):
    if fileinput.lineno() == 2 or fileinput.lineno() == 1:
        print "ignore: " + line
        continue
    line = line.strip(" \n")
    strList = line.split(" ")
    xList.append(float(strList[0]))
    yList.append(float(strList[1]))
    zList.append(float(strList[2]))

    
# read pcd data
pcdData = np.zeros((len(xList), 3), float)
for i, x in enumerate(xList):
    pcdData[i] = [x, yList[i], zList[i]]
# read distance
fDis = open("./b_mds_dist.off", "r")
num_line = 0
for line in fDis.readlines():
    num_line += 1
width = 0
fDis = open("./b_mds_dist.off", "r")
for line in fDis.readlines():
    line = line.strip(" \n")
    strList = line.split(' ')
    width = len(strList)
    break
print num_line
print width
similarities = np.zeros((num_line, width), int)
fDis = open("./b_mds_dist.off", "r")
for i, line in enumerate(fDis.readlines()):
    line = line.strip(" \n")
    strList = line.split(' ')
    l = []
    for d in strList:
        l.append(int(d))
    similarities[i] = l
    l = []
print similarities
print "*****************************************8" 
    
n_samples = 5
seed = np.random.RandomState(seed=3)
#X_true = seed.randint(0, n_samples, 3 * n_samples).astype(np.float)
#X_true = X_true.reshape((n_samples, 3))
# Center the data
#X_true -= X_true.mean()
#similarities = euclidean_distances(pcdData)
print type(similarities)
# run mds algorithm
print "mds pcd data.." 
mds = manifold.MDS(n_components=3, max_iter=100,
                   eps=1e-9, random_state=seed,
                   n_jobs=1)
pos = mds.fit(similarities).embedding_

xList = []
yList = []
zList = []
for p in pos:
    xList.append(p[0])
    yList.append(p[1])
    zList.append(p[2])

print similarities
print "********************888" 
print euclidean_distances(pos)
f = open("out.off","w")
print >> f, "OFF" 
print >> f, "%s 0 0" % len(xList)  
for i,x in enumerate(xList):
    print >> f, "%s %s %s"%(xList[i], yList[i], zList[i])
#"**************************************************************8" 
print " new mds pcd data.." 
nmds = manifold.MDS(n_components=3, metric=False,
                    max_iter=100,
                    eps=1e-9, random_state=seed, n_jobs=1)
npos = nmds.fit_transform(similarities)
# plot the data
fig = plt.figure()
ax = fig.gca(projection='3d')
#xList = []
#yList = []
#zList = []
#for p in X_true:
    #xList.append(p[0])
    #yList.append(p[1])
    #zList.append(p[2])
#ax.scatter(xList, yList, zList, c='b')
#ax.scatter(xList, yList, zList, c='r')
print "********************888" 
print euclidean_distances(npos)
ax.legend()

plt.show()

