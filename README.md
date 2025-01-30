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


Approach:- 

a. Shortest Path Calculation with Velocity Constraints

1. * Grid Creation: A 3D grid is created, with each point either having a random weight (higher cost) or zero (free point).
   * GRID_SIZE: This tuple represents the dimensions of the 3D grid. It spans from (0,0,0) to (100,100,100), with unit increments along each axis (i.e., each coordinate in the grid ranges from 0 to 100 along x, y, and z).
   * The grid is essentially a 3D array with the shape (101, 101, 101) since the points include both endpoints.

Create grid with random weights:-

* np.random.seed(42): This ensures that the random generation is reproducible.
* grid = np.zeros(GRID_SIZE): This initializes the 3D grid with zeros, indicating that initially all points are free to traverse.
* num_high_weight_points = 500: This specifies how many points in the grid should be assigned a high weight (obstacles or higher travel cost).

Assign high weights randomly:-

* for _ in range(num_high_weight_points): This loop runs num_high_weight_points times (500 times in this case), randomly selecting a point in the 3D grid each time.
* x, y, z = np.random.randint(0, 101, 3): For each iteration, three random integers between 0 and 100 (inclusive) are generated, corresponding to the x, y, and z coordinates of the grid point.
* grid[x, y, z] = high_weight_value: The grid point at the random coordinates (x, y, z) is assigned the high weight value (100), representing obstacles or higher travel costs.



2. * A Algorithm*: The shortest path for each set of start and end points is computed using the A algorithm*. This algorithm is particularly efficient for pathfinding in weighted grids.
     
   Heuristic Function:
   * The A* algorithm uses a heuristic (like the Euclidean distance or Manhattan distance) to guide the search towards the goal efficiently.
