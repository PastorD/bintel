#!/usr/bin/env python

import random
import numpy as np

class RBF:

    def __init__(self, dim, numCenters):
        self.dim = dim
        self.numCenters = numCenters
        self.centers = np.random.uniform(-1, 1, (numCenters, dim))

    def thin_plate_spline(self, X):
        assert X.shape[1] == self.dim
        rbf = np.array([np.linalg.norm(X-c, axis=1)**2*np.log(np.linalg.norm(X-c, axis=1)) for c in self.centers]).reshape(self.numCenters, X.shape[0])

        return np.concatenate((X.transpose(), rbf), axis=0)