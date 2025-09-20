"""
@file safe_astar.py
@brief Clearance-aware A* on N-D occupancy grids using EDT-based safety costs.

@details
This module implements an A* planner on an N-D occupancy grid (0 = free,
1 = obstacle) that explicitly trades off path length and safety. Instead of
inflating obstacles, it computes a Euclidean Distance Transform (EDT) over
free space and adds a **soft safety penalty** to each edge cost:

  g(u→v) = step_cost(u, v) + safetyFactor / (epsilon + 0.5*(EDT[u] + EDT[v]))

This penalizes moves that pass close to obstacles while keeping the full
free space available (no hard removal of narrow passages). It differs from
“inflated A*” where near-obstacle cells are marked occupied.

Conventions:
- Grid values: 0 = free, 1 = obstacle.
- Coordinates: integer grid indices in N dimensions.
- Neighborhood: all 3^N - 1 offsets (axis-aligned + diagonals).
- Step costs: axis moves cost 1.0; diagonal/off-axis cost L2 magnitude.
- Heuristic: Manhattan (L1) by default. If you allow diagonal moves and want
  strict admissibility, prefer Chebyshev (L∞) or Euclidean (L2 with consistent
  step costs).

@par Inputs (to plan())
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col[, ...]))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}

@par Constructor Args
- @p safetyFactor : float — weight on the EDT-based safety penalty (larger → more clearance-seeking)
- @p epsilon      : float — small positive constant to avoid division by zero in the safety term

@par Outputs (from plan())
- @p success : int — 1 if a path is found, else 0
- @p path    : list[tuple[int, ...]] — sequence of cells from start→goal (inclusive) or empty
- @p info    : list[str] — diagnostics (e.g., "Invalid goal", "Failed to reconstruct path")

@note
- This “soft-constraint” approach keeps narrow passages feasible but discourages
  them unless necessary. Tune @p safetyFactor and @p epsilon to balance optimality
  vs. clearance.
- With diagonal moves, Manhattan (L1) is not strictly admissible; use L∞ or L2
  with consistent step costs if optimality guarantees are required.

@see BasePlanner, distance_transform_edt

@references
- A* (Hart, Nilsson, Raphael): https://ieeexplore.ieee.org/document/4082128
- EDT (SciPy): https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.distance_transform_edt.html
- Clearance-aware costs (general idea appears in many navigation & cost-inflation methods).
"""

from .baseplanner import BasePlanner
import heapq
import itertools
import numpy as np
from scipy.ndimage import distance_transform_edt


class SafeAStar(BasePlanner):
    """
    @class SafeAStar
    @brief A* with EDT-weighted safety costs (no hard inflation).

    @details
    Pipeline:
      1) Compute EDT on (1 - grid) so free cells carry clearance values.
      2) Use full 3^N - 1 neighborhood for expansion.
      3) Edge cost = step_cost + safetyFactor / (epsilon + avg(EDT(u), EDT(v))).
      4) Run A* and reconstruct the path on success.

    This preserves reachability of narrow corridors while penalizing low-clearance
    motion, often yielding safer paths compared to pure geometric cost.
    """

    def __init__(self, safetyFactor: float, epsilon: float):
        """
        @brief Construct a clearance-aware A* planner.

        @param safetyFactor Weight on the EDT-based safety penalty term.
        @param epsilon      Small positive constant providing numerical stability.
        """
        self.success = 0
        self.safetyFactor = float(safetyFactor)
        self.epsilon = float(epsilon)
        self.info = []
        self.path = []

    # ---------------------------
    # Utility helpers
    # ---------------------------

    def isValid(self, grid_cell) -> bool:
        """
        @brief Check whether a cell lies within grid bounds.

        @param grid_cell tuple[int, ...] Cell index to validate.
        @return True if the cell is inside the grid extent; False otherwise.

        @complexity O(N) with N = grid dimensionality.
        """
        for i in range(self.dimension):
            if not (0 <= grid_cell[i] < self.grid.shape[i]):
                return False
        return True

    def heuristics(self, node1, node2) -> float:
        """
        @brief Manhattan (L1) heuristic between two nodes.

        @param node1 tuple[int, ...]
        @param node2 tuple[int, ...]
        @return float Sum over dimensions of |node1[i] - node2[i]|.

        @note For diagonal-enabled neighborhoods, consider Chebyshev (L∞) or
              Euclidean (L2 with compatible step costs) for strict admissibility.
        """
        cost = 0.0
        for i in range(self.dimension):
            cost += abs(node1[i] - node2[i])
        return float(cost)

    def adjacentCoordinates(self, node):
        """
        @brief Enumerate all adjacent coordinates in N-D (including diagonals).

        @param node tuple[int, ...] The reference grid cell.
        @return list[tuple[int, ...]] All 3^N - 1 neighbor coordinates
                (not yet filtered by bounds/occupancy).

        @details
        Generates offsets in {-1, 0, 1}^N \\ {0} via itertools.product.
        Validity and occupancy checks are performed during expansion in @ref plan().

        @complexity O(3^N).
        """
        offsets = [-1, 0, 1]
        combinations = itertools.product(offsets, repeat=self.dimension)
        adjacent = []
        for c in combinations:
            if all(o == 0 for o in c):
                continue
            nb = tuple(node[i] + c[i] for i in range(self.dimension))
            adjacent.append(nb)
        return adjacent

    # ---------------------------
    # Main planning routine
    # ---------------------------

    def plan(self, start, goal, grid):
        """
        @brief Run clearance-aware A* from @p start to @p goal on @p grid.

        @param start tuple[int, ...] Start index in the N-D grid.
        @param goal  tuple[int, ...] Goal index in the N-D grid.
        @param grid  numpy.ndarray Occupancy grid (0=free, 1=obstacle).

        @return (success, path, info)
          - success (int): 1 if a path is found, else 0
          - path (list[tuple[int, ...]]): sequence from start→goal (inclusive)
          - info (list[str]): diagnostics / failure reasons

        @diagnostics
        - "Invalid start", "Invalid goal"
        - "Start has obstacle", "Goal has obstacle"
        - "Start and goal are same"
        - "Failed to reconstruct path."

        @complexity
        Time: O(E log V) with a binary heap, where V is the number of valid
              cells and E the number of explored edges under the chosen neighborhood.
        Space: O(V) for g_score, visited, and parents.
        """
        # Initialize
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.grid = grid
        self.path = []
        self.info = []
        self.success = 0

        self.dimension = len(start)
        self.sizes = [np.size(grid, axis=i) for i in range(self.dimension)]

        # Input validation
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

        # Clearance map (EDT on free space)
        self.distanceTransform = distance_transform_edt(1 - self.grid)

        # A* data
        heap = []
        visited = np.zeros(self.grid.shape, bool)
        parents = {}
        g_score = np.full(self.grid.shape, np.inf, np.float32)

        g_score[self.start] = 0.0
        f0 = self.heuristics(self.start, self.goal)
        heapq.heappush(heap, (f0, 0.0, self.start))
        self.parent = None
        self.node = self.start

        # Main loop
        while heap:
            _, g, self.node = heapq.heappop(heap)

            if self.node == self.goal:
                break

            if visited[self.node]:
                continue
            visited[self.node] = 1

            # Expand neighbors
            for nb in self.adjacentCoordinates(self.node):
                if not (self.isValid(nb) and self.grid[nb] == 0 and not visited[nb]):
                    continue

                self.parent = self.node

                # Early exit if we reached the goal
                if nb == self.goal:
                    parents[nb] = self.parent
                    self.success = 1
                    break

                # Step cost: axis steps = 1.0, diagonals = L2 magnitude
                delta = tuple(nb[i] - self.node[i] for i in range(self.dimension))
                is_axis_step = sum(abs(d) for d in delta) == 1
                step_cost = 1.0 if is_axis_step else float(np.linalg.norm(delta, ord=2))

                # EDT-based safety term (soft clearance penalty)
                clr_u = self.distanceTransform[self.node]
                clr_v = self.distanceTransform[nb]
                safety = self.safetyFactor / (self.epsilon + 0.5 * (clr_u + clr_v))

                g_updated = g + step_cost + safety
                if g_updated < g_score[nb]:
                    g_score[nb] = g_updated
                    f = g_updated + self.heuristics(nb, self.goal)
                    heapq.heappush(heap, (f, g_updated, nb))
                    parents[nb] = self.parent

            if self.success == 1:
                break

        # Reconstruct path
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
