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

#print __doc__
import numpy as np

from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
import numpy as np
#from matplotlib.collections import LineCollection

from sklearn import manifold
from sklearn.metrics import euclidean_distances


def compute(data):
    '''docstring for compute''' 
    # read pcd data
    similarities = np.zeros((len(data),len(data)), int)
    for i, line in enumerate(data):
        for j, item in enumerate(line):
           similarities[i][j] = item; 
        
    seed = np.random.RandomState(seed=3)
    #X_true = seed.randint(0, n_samples, 3 * n_samples).astype(np.float)
    #X_true = X_true.reshape((n_samples, 3))
    # Center the data
    #X_true -= X_true.mean()
    #similarities = euclidean_distances(pcdData)
    #print "similarities:" 
    #print similarities
    # run mds algorithm
    mds = manifold.MDS(n_components=2, max_iter=300,
                       eps=1e-9, random_state=seed,
                       n_jobs=1)
    pos = mds.fit(similarities).embedding_
    #print "should equal to similarities:" 
    #print euclidean_distances(pos)
    #"**************************************************************8" 
    #print " new mds pcd data.." 
    #nmds = manifold.MDS(n_components=3, metric=False,
                        #max_iter=100,
                        #eps=1e-9, random_state=seed, n_jobs=1)
    #npos = nmds.fit_transform(similarities)
    #print "should equal to similarities:" 
    #print euclidean_distances(npos)
    # plot the data
    return pos

