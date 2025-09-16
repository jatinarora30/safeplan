"""
@file voronoi_planner.py
@brief Voronoi-roadmap planner for N-dimensional occupancy grids.

@details
Builds a roadmap from the Voronoi diagram of obstacle clusters and searches it
with A* to obtain a collision-free path between a start and a goal cell on an
N-D NumPy occupancy grid (0 = free, 1 = obstacle). The planner first checks
whether the straight line from start to goal is collision-free; if so, it returns
that two-point path. Otherwise, it:
  1) segments obstacles via connected-component labeling and computes each
     component's centroid as a Voronoi site,
  2) constructs the Voronoi diagram (finite edges only),
  3) builds a graph from collision-free Voronoi edges,
  4) connects start and goal to their K nearest Voronoi vertices if collision-free,
  5) runs A* on this graph to produce a piecewise-linear path.

@par Inputs
- @p start : tuple[int, ...] — start grid cell (for example, (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}

@par Outputs
- @p success : int — 1 if a path is found, else 0
- @p path    : list[tuple[int, ...]] — sequence of cells from start to goal (inclusive)
- @p info    : list[str] or str — diagnostics (for example, "Invalid goal") or a success note

@note
- Requires a sufficient number of obstacle components to form a meaningful
  Voronoi diagram (this implementation expects more than four).
- Collision checks on edges sample @ref pointSamples evenly spaced points and
  round to the nearest grid indices for occupancy testing.
- Grid convention: 1 = obstacle, 0 = free.

@see BasePlanner

@references
- SciPy Voronoi: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.Voronoi.html
- SciPy ndimage.label: https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.label.html
- NetworkX A*: https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.shortest_paths.astar.astar_path.html
"""

from .baseplanner import BasePlanner
from scipy.spatial import Voronoi
import numpy as np
import networkx as nx
from scipy.ndimage import label


class VoronoiPlanner(BasePlanner):
    """
    @class VoronoiPlanner
    @brief Path planner using a Voronoi roadmap with A* search.

    @details
    The planner assembles a sparse roadmap that tends to maximize clearance from
    obstacles by leveraging the Voronoi diagram of obstacle-cluster centroids.
    Start and goal are connected to nearby Voronoi vertices if those connections
    are collision-free, and the shortest path on that graph (by Euclidean edge
    weights) is computed via A* with a Euclidean heuristic.

    @var success
        (int) Planning outcome flag (1 on success, 0 on failure).
    @var pointSamples
        (int) Number of samples per edge for collision checking.
    @var knn
        (int) Number of nearest Voronoi vertices to connect for start and goal.
    @var info
        (list|str) Diagnostics or status for the last plan call.
    @var path
        (list[tuple[int, ...]]) Planned path as grid coordinates.
    """

    def __init__(self, pointSamples, knn):
        """
        @brief Construct the Voronoi planner.

        @param pointSamples int
               Number of interpolation samples used when checking whether a
               straight-line edge is free of obstacles.
        @param knn int
               Number of nearest Voronoi vertices to attempt to connect for
               both start and goal.

        @post Instance is initialized with empty path and diagnostics.
        """
        self.success = 0
        self.pointSamples = pointSamples
        self.knn = knn
        self.info = []

        self.path = []

    def getObstacleCenters(self, grid):
        """
        @brief Compute centroids of connected obstacle components.

        @details
        Uses `scipy.ndimage.label` to identify connected components (features)
        of obstacle cells (grid == 1). For each component, computes the mean of
        its indices as the centroid, expressed in continuous coordinates.

        @param grid numpy.ndarray
               N-D occupancy grid with {0=free, 1=obstacle}.

        @return numpy.ndarray
                Array of shape (M, N) with centroids of M components in N dims.
                Returns an empty array if no obstacles are present.
        """
        labeled_grid, num_features = label(grid)
        centers = []
        for label_num in range(1, num_features + 1):
            positions = np.argwhere(labeled_grid == label_num)
            center = positions.mean(axis=0)
            centers.append(center)
        return np.array(centers)

    def isEdgeFree(self, pt1, pt2):
        """
        @brief Check if the segment between two points is collision-free.

        @details
        Samples `self.pointSamples` points linearly between @p pt1 and @p pt2.
        Each sample is rounded to the nearest grid index; if any sample is
        out of bounds or falls on an obstacle cell (grid == 1), the edge is
        considered in collision.

        @param pt1 sequence[float]
               Segment start (continuous coordinates).
        @param pt2 sequence[float]
               Segment end (continuous coordinates).

        @return bool
                True if all sampled points lie within the grid and in free
                cells; otherwise False.
        """
        for t in np.linspace(0, 1, self.pointSamples):
            point = (1 - t) * np.array(pt1) + t * np.array(pt2)

            valid = True
            for p in range(self.dimension):
                if not (0 <= point[p] < self.grid.shape[p]):
                    valid = False
                    break
                if valid:
                    point2 = []
                    for k in range(self.dimension):
                        point2.append(int(round(point[k])))
                    if self.isValid(tuple(point2)):
                        if self.grid[tuple(point2)] == 1:
                            return False
        return True

    def isValid(self, grid_cell):
        """
        @brief Check whether a grid-cell index lies within bounds.

        @param grid_cell tuple[int, ...]
               N-dimensional integer grid index.

        @return bool
                True if all indices are within the grid extents; otherwise False.
        """
        for i in range(self.dimension):
            if not (0 <= grid_cell[i] < self.grid.shape[i]):
                return False
        return True

    def nearestVertex(self, point):
        """
        @brief Return indices of the K nearest Voronoi vertices to a query point.

        @details
        Computes Euclidean distance from @p point to each finite Voronoi vertex
        and returns up to `self.knn` indices of the closest ones.

        @param point sequence[float]
               Query location in continuous coordinates.

        @return list[int]
                Indices of up to K nearest Voronoi vertices.
        """
        vertexes = []

        for idx, vertex in enumerate(self.voronoiPoints.vertices):
            if np.any(np.isnan(vertex)):
                continue
            dist = np.linalg.norm(np.array(vertex) - np.array(point))
            vertexes.append((idx, dist))
        vertexes.sort(key=lambda x: x[1])
        nearestIdxs = []
        for idx, dist in vertexes[:self.knn]:
            nearestIdxs.append(idx)

        return nearestIdxs

    def plan(self, start, goal, grid):
        """
        @brief Plan a path from @p start to @p goal over @p grid via a Voronoi roadmap.

        @details
        Algorithm flow:
          1. Validate inputs and trivial cases (invalid cells, obstacles, equality).
          2. Attempt a straight-line path; if collision-free, return immediately.
          3. If enough obstacle components exist (more than four), build the
             Voronoi diagram from obstacle-centroid sites.
          4. Construct a graph whose nodes are Voronoi vertices (plus start and goal)
             and whose edges are Voronoi edges that pass collision checks.
          5. Connect start and goal to their @ref knn nearest Voronoi vertices if
             those connections are collision-free.
          6. Run A* (NetworkX) with Euclidean edge weights and heuristic to obtain
             the shortest path on the roadmap; convert to integer grid positions.

        @param start tuple[int, ...]
               Start grid cell.
        @param goal tuple[int, ...]
               Goal grid cell.
        @param grid numpy.ndarray
               N-D occupancy grid with {0=free, 1=obstacle}.

        @return success int
                1 if a path is found, else 0.
        @return path list[tuple[int, ...]]
                Path from start to goal (inclusive) when found; else empty or 0.
        @return info list[str] or str
                Diagnostics on failure or a brief status message on success.

        @throws None
                Any networkx no-path exception is caught and encoded in the return values.
        """
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.grid = grid
        self.path = []
        self.info = []
        self.success = 0
        self.dimension = len(start)
        self.sizes = []
        for i in range(0, self.dimension):
            self.sizes.append(np.size(grid, axis=i))

        if not self.isValid(self.start):
            self.info.append("Invalid start ")
            return self.success, self.path, self.info

        if not self.isValid(self.goal):
            self.info.append("Invalid  goal")
            return self.success, self.path, self.info

        if self.grid[self.goal] == 1:
            self.info.append("Goal has obstacle")
            return self.success, self.path, self.info

        if self.grid[self.start] == 1:
            self.info.append("Start has obstacle")
            return self.success, self.path, self.info

        if self.start == self.goal:
            self.info.append("Start and goal are same")
            return self.success, self.path, self.info
        obstacles = []

        obstacles = self.getObstacleCenters(grid)
        if self.isEdgeFree(self.start, self.goal):
            self.success = 1
            self.info = "Straight line Path"
            self.path = [self.start, self.goal]

            return self.success, self.path, self.info
        if len(obstacles) > 4:
            self.voronoiPoints = Voronoi(obstacles)
            idx = 0
            self.G = nx.Graph()
            for idx, vertex in enumerate(self.voronoiPoints.vertices):
                self.G.add_node(idx, pos=tuple(vertex))

            idx += 1

            self.startID = idx
            self.G.add_node(idx, pos=self.start)
            idx += 1
            self.goalID = idx
            self.G.add_node(idx, pos=self.goal)

            for ridge_vertices in self.voronoiPoints.ridge_vertices:
                if -1 in ridge_vertices:
                    continue  # Ignore infinite ridges
                idx1, idx2 = ridge_vertices
                pt1 = self.voronoiPoints.vertices[idx1]
                pt2 = self.voronoiPoints.vertices[idx2]
                if self.isEdgeFree(pt1, pt2):
                    distance = np.linalg.norm(pt1 - pt2)
                    self.G.add_edge(idx1, idx2, weight=distance)

            nearestToStarts = self.nearestVertex(self.start)
            for nearestToStart in nearestToStarts:
                if nearestToStart is not None and self.isEdgeFree(self.start, self.voronoiPoints.vertices[nearestToStart]):
                    distance = np.linalg.norm(self.start - self.voronoiPoints.vertices[nearestToStart])
                    self.G.add_edge(self.startID, nearestToStart, weight=distance)

            nearestToGoals = self.nearestVertex(self.goal)

            for nearestToGoal in nearestToGoals:
                if nearestToGoal is not None and self.isEdgeFree(self.goal, self.voronoiPoints.vertices[nearestToGoal]):
                    distance = np.linalg.norm(self.goal - self.voronoiPoints.vertices[nearestToGoal])
                    self.G.add_edge(self.goalID, nearestToGoal, weight=distance)

            try:
                path = nx.astar_path(
                    self.G,
                    self.startID,
                    self.goalID,
                    heuristic=lambda a, b: float(
                        np.linalg.norm(
                            np.asarray(self.G.nodes[a]['pos'], dtype=float) -
                            np.asarray(self.G.nodes[b]['pos'], dtype=float)
                        )
                    ),
                    weight='weight'
                )

                path_positions = [tuple(np.asarray(self.G.nodes[int(n)]['pos'], dtype=float).reshape(-1)) for n in path]

                # Convert to integer grid positions
                path_grid = [tuple(map(int, pos)) for pos in path_positions]
                self.path = path_grid
                self.success = 1
            except nx.NetworkXNoPath:
                self.success = 0
                self.path = []
                self.info = "Path not found"
        else:
            self.info = "Very less objects to draw vornoi"
            self.success = 0
            self.path = 0

        return self.success, self.path, self.info
