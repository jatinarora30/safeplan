"""
@file fs_planner.py
@brief N-dimensional A* (A-star) planner over NumPy occupancy grids.

@details
This module implements A* search on an N-D occupancy grid where `0` denotes free
space and `1` denotes an obstacle. The planner accepts start and goal grid indices
(tuples of ints) and returns a collision-free path if one exists.

- Heuristic: By default, this implementation uses the **Manhattan (L1)** distance.
- Neighborhood: All 3^N - 1 offsets are considered (axis-aligned and diagonal moves).
- Costs: Axis-aligned steps cost 1.0; diagonal/off-axis steps use the L2 magnitude
  of the step vector.

@attention Heuristic admissibility depends on the move set. If diagonal moves are
allowed (as in this implementation), Manhattan distance is **not** strictly
admissible unless you adjust step costs accordingly. Prefer **Chebyshev** (L∞) or
Euclidean (L2 with consistent step costs) for diagonal-enabled neighborhoods.

@par Inputs
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col, ...))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}

@par Outputs
- @p success : int — 1 if a path is found, else 0
- @p path    : list[tuple[int, ...]] — cells from start→goal (inclusive) or empty if none
- @p info    : list[str] — diagnostics (e.g., "Invalid goal")

@note Ensure the heuristic matches the move set:
      - 4-neighbors (axis-only) → Manhattan (L1)
      - 8-neighbors / diagonal moves → Chebyshev (L∞) or Euclidean (L2 with proper step costs)

@see BasePlanner

@references
- @cite HartNilsNilsson1968  N. J. Nilsson, P. E. Hart, B. Raphael, "A Formal Basis for the
  Heuristic Determination of Minimum Cost Paths", 1968. DOI/Record: https://ieeexplore.ieee.org/document/4082128
"""

from .baseplanner import BasePlanner
import heapq
import itertools
import numpy as np
from scipy.ndimage import distance_transform_edt


class FSPlanner(BasePlanner):
    """
    @class AStar
    @brief A* planner operating on N-D occupancy grids.

    @details
    Provides a grid-based A* implementation with:
    - Full 3^N - 1 neighborhood (axis + diagonal) expansion,
    - Manhattan (L1) heuristic by default (see @ref dist),
    - Axis steps costing 1.0 and diagonal/off-axis steps using L2 magnitude.

    @note For theoretical optimality guarantees, the heuristic must be
          both admissible and consistent with the chosen move set and step costs.
    """

    def __init__(self,pointSamples,cw,epsilon,maxNeigh):
        """
        @brief Construct an A* planner instance.
        @post The planner is initialized with empty results and diagnostics.
        """
        self.success = 0
        self.pointSamples=pointSamples
        self.cw=cw
        self.eps=epsilon
        self.maxNeigh=maxNeigh
        self.info = []
        self.path = []

    def isValid(self, grid_cell):
        """
        @brief Check whether a cell lies within grid bounds.
        @param[in] grid_cell tuple[int, ...] Cell index to validate.
        @return bool True if the cell is inside the grid extent; False otherwise.
        @pre @c self.grid and @c self.dimension are set (by @ref plan()).
        @complexity O(N) with N = grid dimensionality.
        """
        for i in range(self.dimension):
            if not (0 <= grid_cell[i] < self.grid.shape[i]):
                return False
        return True
    def isEdgeFree(self, pt1, pt2):
        for t in np.linspace(0.0, 1.0, self.pointSamples):
            point = (1 - t) * np.array(pt1, float) + t * np.array(pt2, float)
            idx = tuple(int(round(x)) for x in point)
            if not self.isValid(idx) or self.grid[idx] == 1:
                return False
        return True
    
    def EDFDeriv(self,node1,node2):
        
        return (self.distanceTransform[node1]-self.distanceTransform[node2])/(self.dist(node1,node2)+ self.eps)

    def EDFCost(self,node1,node2):
        return self.cw/((((self.distanceTransform[node1]+self.distanceTransform[node2])*self.dist(node1,node2))/2)+self.eps)

    def getEDFNeigh(self, adjacentNodes, distVec):
        node_np = np.array(self.node, float)
        scores = {}
        for nb in adjacentNodes:
            step = np.array(nb, float) - node_np           # use step vector
            n = np.linalg.norm(step) + 1e-9
            step /= n
            scores[nb] = float(np.dot(step, distVec))      # align with desired direction

        # sort by alignment (descending) and KEEP the result
        sorted_nbs = sorted(scores.items(), key=lambda kv: -kv[1])
        return [nb for nb, _ in sorted_nbs[:self.maxNeigh]]


    def adjacentCoordinates(self, node):
        """
        @brief Enumerate all adjacent coordinates in N-D (including diagonals).
        @param[in] node tuple[int, ...] The reference grid cell.
        @return list<tuple[int, ...]> All 3^N - 1 neighbor coordinates (not filtered by validity).
        @details
        Generates all offset combinations in {-1, 0, 1}^N \\ {0} using @c itertools.product.
        Validity and obstacle checks are performed during expansion in @ref plan().
        @complexity O(3^N).
        """
        offsets = [-1, 0, 1]
        combinations = itertools.product(offsets, repeat=self.dimension)
        adjacentNodes = []
        for c in combinations:
            if all(o == 0 for o in c):
                continue
            nb = tuple(node[i] + c[i] for i in range(self.dimension))
            adjacentNodes.append(nb)
        return adjacentNodes
    
    def dist(self, a, b):
        return float(np.linalg.norm(np.subtract(a, b), ord=2))
    
    def _cost(self, a, b):
        return self.dist(a, b) + self.EDFCost(a, b)


    def plan(self, start, goal, grid):
        """
        @brief Run A* planning from @p start to @p goal on @p grid.
        @param[in] start tuple[int, ...] Start index in the N-D grid.
        @param[in] goal  tuple[int, ...] Goal index in the N-D grid.
        @param[in] grid  numpy.ndarray Occupancy grid (0=free, 1=obstacle).
        @return tuple
                - @c success (int) : 1 if a path is found, else 0
                - @c path (list[tuple[int, ...]]) : sequence from start→goal (inclusive)
                - @c info (list[str]) : diagnostics / failure reasons
        @par Diagnostics
        - "Invalid start", "Invalid goal"
        - "Start has obstacle", "Goal has obstacle"
        - "Start and goal are same"
        - "Failed to reconstruct path."
        @note Path reconstruction uses the @c parents map built during search.
        @warning Heuristic/step-cost mismatch can break optimality with diagonal moves.
        @complexity
        Time: O(E log V) using a binary heap, where V is the number of valid cells and
              E is the number of edges explored under the chosen neighborhood.
        Space: O(V) for @c g_score, @c visited, and @c parents.
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

        # Input validation and trivial cases
        if not self.isValid(self.start):
            self.info.append("Invalid start")
            return self.success, self.path, self.info

        if not self.isValid(self.goal):
            self.info.append("Invalid goal")
            return self.success, self.path, self.info

        if self.grid[self.goal] == 1:
            self.info.append("Goal has obstacle")
            return self.success, self.path, self.info

        if self.grid[self.start] == 1:
            self.info.append("Start has obstacle")
            return self.success, self.path, self.info

        if self.start == self.goal:
            self.info.append("Start and goal are same")
            self.success = 1
            self.path = [self.start]
            return self.success, self.path, self.info

        # A* initialization
        heap = []
        visited = np.zeros(self.grid.shape, bool)
        parents = {}
        g_score = np.full(self.grid.shape, np.inf, np.float32)
        g_score[self.start] = 0
        f0 = self.dist(self.start, self.goal)
        heapq.heappush(heap, (f0, 0, self.start))
        self.parent = None
        self.node = self.start
        parents[self.start]=self.start
        self.distanceTransform=distance_transform_edt(1-self.grid)
        # Main loop
        # Main loop (Lazy Theta*)
        while heap:
            _, g_cur, self.node = heapq.heappop(heap)

            if self.node == self.goal:
                self.success = 1
                break

            if visited[self.node]:
                continue
            visited[self.node] = 1

            # Generate & prefilter neighbors
            adjacentNodes = self.adjacentCoordinates(self.node)
            adjacentNodes = [nb for nb in adjacentNodes
                            if self.isValid(nb) and self.grid[nb] == 0 and not visited[nb]]

            goal_dir = np.array(self.goal, float) - np.array(self.node, float)
            goal_dir /= (np.linalg.norm(goal_dir) + 1e-9)

            if adjacentNodes:
                sdf_deriv = {adj: self.EDFDeriv(self.node, adj) for adj in adjacentNodes}
                best_sdf_nb = min(sdf_deriv, key=sdf_deriv.get)
                sdf_dir = np.array(best_sdf_nb, float) - np.array(self.node, float)
                sdf_dir /= (np.linalg.norm(sdf_dir) + 1e-9)
                distVec = goal_dir + sdf_dir
                distVec /= (np.linalg.norm(distVec) + 1e-9)
                adjacentNodes = self.getEDFNeigh(adjacentNodes, distVec)
            # ----------------------------------------------------------------------

            # Lazy Theta* neighbor relaxation
            pu = parents.get(self.node, self.node)  # parent(s)
            for nb in adjacentNodes:
                # Try to connect nb directly to parent(s)
                if self.isEdgeFree(pu, nb):
                    # UpdateVertex LOS case: via parent(s)
                    g_cand = g_score[pu] + self._cost(pu, nb)
                    parent_cand = pu
                else:
                    # UpdateVertex no-LOS case: via s
                    g_cand = g_score[self.node] + self._cost(self.node, nb)
                    parent_cand = self.node

                if g_cand < g_score[nb]:
                    g_score[nb] = g_cand
                    parents[nb] = parent_cand
                    f = g_cand + self.dist(nb, self.goal)  # admissible Euclidean h
                    heapq.heappush(heap, (f, g_cand, nb))

            if self.success == 1:
                break

        # Reconstruct path (if found)
        if self.success == 1:
            path_node = self.goal
            while path_node is not None and path_node != self.start:
                self.path.append(path_node)
                path_node = parents.get(path_node)
            if path_node != self.start:
                self.info.append("Failed to reconstruct path.")
                self.success, self.path = 0, []
                return self.success, self.path, self.info
            self.path.append(self.start)
            self.path.reverse()

        return self.success, self.path, self.info
