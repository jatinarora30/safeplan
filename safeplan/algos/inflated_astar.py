"""
@file inflated_astar.py
@brief Inflated A* (clearance-augmented) planner on N-D occupancy grids.

@details
Implements a grid-based A* that first inflates obstacles by a user-specified
clearance radius, then plans on the inflated grid. Inflation is performed via
a Euclidean Distance Transform (EDT) computed on free space (1 - grid). Any
cell whose EDT value is less than or equal to @p radiusInflate is marked as
occupied in the inflated grid. This biases paths to maintain a minimum
clearance from obstacles without modifying the A* edge cost itself.

Conventions:
- Grid values: 0 = free, 1 = obstacle.
- Coordinates are integer grid indices in N dimensions.
- Neighborhood: all 3^N - 1 offsets (axis-aligned + diagonals).
- Step costs: axis-aligned steps cost 1.0; diagonal/off-axis steps use the L2
  magnitude of the step vector.
- Heuristic: Manhattan (L1) by default. For diagonal moves, prefer Chebyshev (L∞)
  or Euclidean (L2 with consistent edge costs) to preserve admissibility.

@par Inputs (to plan())
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col[, ...]))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}

@par Constructor Args
- @p radiusInflate : float — minimum clearance radius (in grid cells) enforced by obstacle inflation

@par Outputs (from plan())
- @p success : int — 1 if a path is found, else 0
- @p path    : list[tuple[int, ...]] — sequence of cells from start→goal (inclusive) or empty
- @p info    : list[str] — diagnostics (e.g., "Invalid goal", "Failed to reconstruct path")

@note
- This method enforces safety by *removing* near-obstacle cells from the search
  space rather than by penalizing costs. It may render some problems infeasible
  if @p radiusInflate is too large for narrow passages.
- If you keep diagonal moves with Manhattan heuristic, A* is no longer strictly
  admissible. Use Chebyshev or Euclidean if optimality guarantees matter.

@see BasePlanner, distance_transform_edt

@references
- Inflated A*: https://arxiv.org/pdf/2003.00368 
"""

from .baseplanner import BasePlanner
import heapq
import itertools
import numpy as np
from scipy.ndimage import distance_transform_edt


class InflatedAStar(BasePlanner):
    """
    @class InflatedAStar
    @brief A* planner that runs on an EDT-inflated occupancy grid.

    @details
    Pipeline:
      1) Compute EDT on (1 - grid) so free cells carry positive clearance.
      2) Inflate obstacles by thresholding EDT at @p radiusInflate.
      3) Run A* on the inflated grid with full 3^N - 1 neighborhood.
      4) Reconstruct the path if the goal is reached.

    This strategy is a simple way to enforce a clearance margin using standard A*.
    """

    def __init__(self, radiusInflate: float):
        """
        @brief Construct an inflated-grid A* planner.

        @param radiusInflate Minimum clearance radius in *grid cells*.
                              Any cell with EDT <= radiusInflate is treated as occupied.
        """
        self.success = 0
        self.radiusInflate = float(radiusInflate)
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

        @note When allowing diagonal moves, consider Chebyshev (L∞) or Euclidean
              (L2 with compatible step costs) to maintain admissibility.
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
        Generates all offset combinations in {-1, 0, 1}^N \\ {0} using itertools.product.
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
    # Grid inflation
    # ---------------------------

    def inflateGrid(self) -> np.ndarray:
        """
        @brief Inflate obstacles using an EDT-based clearance threshold.

        @details
        Computes distance_transform_edt on (1 - grid), where free cells are 1.
        Any cell whose EDT <= @p radiusInflate is set to 1 (occupied) in the
        inflated grid.

        @return np.ndarray Inflated occupancy grid (0=free, 1=inflated obstacle).
        """
        # EDT on free space: 1 for free, so distance increases away from obstacles.
        self.distanceTransform = distance_transform_edt(1 - self.grid)
        inflated = (self.grid == 1) | (self.distanceTransform <= self.radiusInflate)
        return inflated.astype(int)

    # ---------------------------
    # Main planning routine
    # ---------------------------

    def plan(self, start, goal, grid):
        """
        @brief Run Inflated A* from @p start to @p goal on @p grid.

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
        Time: O(E log V) using a binary heap, where V is number of valid cells and
              E is number of explored edges under the chosen neighborhood.
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

        # Inflate grid to enforce minimum clearance constraint
        self.grid = self.inflateGrid()

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

                # Early exit if we reached the goal
                self.parent = self.node
                if nb == self.goal:
                    parents[nb] = self.parent
                    self.success = 1
                    break

                # Step cost: axis steps = 1.0, diagonals = L2 magnitude
                delta = tuple(nb[i] - self.node[i] for i in range(self.dimension))
                is_axis_step = sum(abs(d) for d in delta) == 1
                step_cost = 1.0 if is_axis_step else float(np.linalg.norm(delta, ord=2))

                g_updated = g + step_cost
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
