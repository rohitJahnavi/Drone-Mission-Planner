# 3D Grid-Based Shortest Path Algorithm with Velocity Constraints

Overview:-

This repository implements a 3D grid-based shortest path algorithm where multiple paths are calculated for different start and end points on a 3D grid. The grid spans from (0,0,0) to (100,100,100) with unit increments in all axes. The grid points are assigned random weights (higher weights for certain points and zero for others) representing obstacles or higher travel costs.

Key Features:

* 3D Grid: A 3D grid with coordinates from (0,0,0) to (100,100,100).
* Weighted Grid: Some grid points have higher weights representing obstacles or costs of travel.
* Multiple Paths: The algorithm handles multiple start and end points.
* Velocity Constraints: Paths are calculated with the condition that they don't intersect at the same time.
* 3D Visualization: The paths are plotted in 3D space to visualize the results.

Problem Definition:-

* Grid: A 3D grid with coordinates spanning from (0,0,0) to (100,100,100) with unit increments along each axis. Some grid points have a weight (higher travel cost), while others have zero weight (free to move).
* Start/End Points: The user inputs multiple start and end points. Each pair represents the start and destination of a path.
* Velocity: The paths are calculated considering a fixed velocity (v m/s). The paths must not overlap at any time, ensuring no collisions between multiple paths.
* Shortest Path Algorithm: The algorithm computes the shortest path for each set of start and end points. It accounts for both the weight of the grid and the constraint that no two paths share the same point at the same time.

import numpy as np

* This imports the NumPy library and assigns it the alias np.
* NumPy provides efficient array operations and mathematical functions, making it useful for numerical computations.
  
import heapq

* This imports the heapq module, which provides a priority queue (min-heap) implementation.
* It is useful for implementing algorithms like Dijkstra’s shortest path algorithm or A search*, where we need to efficiently retrieve the smallest element.

  
import matplotlib.pyplot as plt

* This imports the pyplot module from the matplotlib library and assigns it the alias plt.
* matplotlib.pyplot is used for plotting 2D and 3D graphs, creating visualizations, and displaying data.

  
from mpl_toolkits.mplot3d import Axes3D

* This imports Axes3D from mpl_toolkits.mplot3d, enabling 3D plotting with Matplotlib.
* Without this, matplotlib cannot render 3D plots.
