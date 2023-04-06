#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d

points = np.array([[3, 2], [0, 4.5], [3, 7], [1.5, 9.5]])
vor = Voronoi(points)

fig = voronoi_plot_2d(vor)
plt.show()