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
   * Purpose: This function calculates the Euclidean distance between two points a and b in the grid. The Euclidean distance is used as the heuristic for the A* algorithm, which helps estimate the cost to reach the goal from any given point.
   * np.linalg.norm: Computes the magnitude (or distance) between two points in 3D space, which is useful for estimating the remaining distance in the A* algorithm.
   * neighbors: These are the relative coordinates of neighboring points in the 3D grid (6 possible directions: x, y, z).
   * open_set: A priority queue (min-heap) to hold the points to explore, prioritized by their f_score. It starts with the starting point.
   * came_from: A dictionary to trace the path by storing where each point came from.
   * g_score: A dictionary that keeps track of the cost to reach each point. Initially, the starting point has a score of 0.
   * f_score: A dictionary that stores the sum of the g_score and the heuristic estimate (f_score = g_score + heuristic).
  
* Main Loop (Pathfinding)

  * Priority Queue: The algorithm pops the point with the lowest f_score from the open_set.
  * Goal Check: If the current point equals the goal, the path is reconstructed by backtracking through the came_from dictionary.
  * Path Reconstruction: The path is built by following the came_from dictionary, starting from the goal and moving backwards to the start. It is then reversed to return the correct order from start to goal.

* Exploring Neighbors

   * Neighbors Exploration: For each neighbor of the current point, the algorithm calculates its tentative g-score (cost to reach this neighbor). This is the current g-score plus the cost of moving to the neighbor (1 for normal points, plus the weight of the neighbor).
   * Boundary Check: It ensures that the neighbor is within the bounds of the grid.
   * Update Scores: If a neighbor hasn't been visited yet, or if the new path to it is better (lower g-score), the algorithm updates the scores and adds the neighbor to the priority queue for further exploration.
 
3. * No Path Found

   *  Multiple sets of start and end points are processed sequentially, ensuring that paths do not interfere with each other at the same time.
   * Return None: If the algorithm exhausts the open set without reaching the goal, it means no path exists from the start to the goal.

 b. 3D Visualization of Paths
     
   * Once the paths are calculated, the results are visualized in 3D using Matplotlib and mpl_toolkits.mplot3d. The paths are plotted in different colors or markers to distinguish between them.




* Create the Dictionary of 15 Waypoints:
   
     * Let's start by defining a dictionary for 15 waypoints, each containing 'lat', 'lon', and 'alt'.
     * These waypoints are just placeholders (San Francisco coordinates), and you can adjust them according to your needs.

* Convert Waypoints into DroneKit Locations:
  
     * To convert your waypoints into LocationGlobalRelative objects (which DroneKit uses for mission waypoints), you can iterate through your list of waypoints and create a list of LocationGlobalRelative objects.


* Planning the Drone Mission :

     * This section of the code is responsible for planning the mission by clearing any existing mission commands, adding new mission items (waypoints), and uploading the updated mission to the vehicle. This ensures that the drone follows a set of predefined waypoints.



* Inserting a New Waypoint After 10 Waypoints :

     * This section of the project adds a new waypoint 100 meters perpendicular to the current direction of travel after the drone has passed the first 10 waypoints. In this example, the new waypoint is placed 100 meters to the east of the 10th waypoint. After inserting the new waypoint, the mission is uploaded to the vehicle, updating the flight plan.


* Starting the Drone Mission:
  
     * This section of the code is responsible for starting the mission by arming the drone and setting it to AUTO mode, which allows the drone to autonomously follow the mission waypoints.



* Monitoring Drone Mission Progress:

   * This part of the project continuously monitors the progress of the drone as it moves through the waypoints, updating the distance to the next waypoint and printing the progress at regular intervals.


* 2D Path Visualization of Drone Mission

     * This section of the project handles the visualization of the drone's flight path in 2D. The path is plotted on a longitude vs latitude plane, showing the route the drone will take across all waypoints.


* Steps to Plot the 2D Path:

  1. Extract Latitude and Longitude: The waypoints dictionary contains the geographic coordinates (latitude and longitude) of the drone's waypoints. We extract the latitudes and longitudes to plot them.
  2. Plot the Path: Using Matplotlib, we plot the flight path. Each waypoint is represented by a point on the plot, and the path between them is shown as a connected line.
     * 'bo-': The 'b' represents blue color for the path, the 'o' represents circular markers for the waypoints, and the '-' connects the waypoints with a line.
     * label: The label "Flight Path" will appear in the legend for the plot
  3. Add Labels and Title: The axes are labeled, and a title is added to the plot to make it clearer.
  4. Display the Legend: The legend is displayed to differentiate the flight path, especially if multiple paths are plotted.
  5. Show the Plot: Finally, the plot is shown with plt.show(), which renders the 2D path visualization.
  6. Close the Vehicle Connection: After the mission and plotting are complete, the connection to the vehicle is closed.

