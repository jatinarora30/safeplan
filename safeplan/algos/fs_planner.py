"""
@file fs_planner.py
@brief FS-Planner: Fast and Safe path planning with SDF guidance and Lazy Theta*.

@details
Implements FS-Planner over an N-D NumPy occupancy grid (0 = free, 1 = obstacle).
FS-Planner augments A* with:
  - Signed Distance Field (SDF) based safety-aware edge costs,
  - SDF-derivative guidance to prefer safer corridors,
  - Lazy Theta* line-of-sight (LOS) parent connections to reduce expansions,
  - Adaptive neighbor pruning toward a blended safety/goal direction.

Grid cells are addressed in index space. Start and goal are N-D integer tuples.

Key behaviors:
  - Neighborhood: full 3^N - 1 offsets (axis-aligned and diagonals).
  - Heuristic: Euclidean distance to the goal.
  - Step cost: geometric distance, plus an SDF-based clearance penalty.
  - Collision checking along edges via uniform samples.

FS-Planner trades strict shortest-path optimality for higher clearance and
smoother, lower-risk routes. Tuning @p cw adjusts this trade-off.

@par Inputs (to plan())
- @p start : tuple[int, ...] — start grid cell
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), occupancy {0=free, 1=obstacle}

@par Constructor Args
- @p pointSamples : int — samples per edge for collision checks
- @p cw           : float — clearance weight in edge cost
- @p epsilon      : float — small constant for numerical stability
- @p maxNeigh     : int — cap on neighbors kept after adaptive pruning

@par Outputs (from plan())
- @p success : int — 1 if a path is found, else 0
- @p path    : list[tuple[int, ...]] — start→goal (inclusive) or empty
- @p info    : list[str] — diagnostics (e.g., "Invalid goal")

@note
- Heuristic optimality guarantees do not strictly hold with clearance penalties.
- Set cw=0 to approximate baseline A* (still with LOS smoothing if unmodified).
- Distance transform is computed on (1 - grid) so free cells have positive distance.

@see BasePlanner

@references
- FS-Planner paper: https://arxiv.org/pdf/2505.24024
"""

from .baseplanner import BasePlanner
import heapq
import itertools
import numpy as np
from scipy.ndimage import distance_transform_edt


class FSPlanner(BasePlanner):
    """
    @class FSPlanner
    @brief Fast and Safe planner with SDF-guided costs and Lazy Theta*.

    @details
    Enhances A* by:
      - Using a signed-distance-like map (via EDT on free space) to penalize
        traversal near obstacles,
      - Projecting neighbor expansion toward a blended target direction formed
        by goal direction and safest local direction,
      - Attempting direct LOS connections to the parent's parent (Lazy Theta*)
        to reduce expansions and path zig-zags.
    """

    def __init__(self, pointSamples: int, cw: float, epsilon: float, maxNeigh: int):
        """
        @brief Construct an FSPlanner instance.

        @param pointSamples Number of interpolation samples for edge collision checks.
        @param cw Clearance weight in the edge cost.
        @param epsilon Small constant to avoid division by zero and improve stability.
        @param maxNeigh Maximum number of neighbors retained after pruning.
        """
        self.success = 0
        self.pointSamples = int(pointSamples)
        self.cw = float(cw)
        self.eps = float(epsilon)
        self.maxNeigh = int(maxNeigh)

        self.info = []
        self.path = []

    # ---------------------------
    # Basic helpers and geometry
    # ---------------------------

    def isValid(self, grid_cell) -> bool:
        """
        @brief Check whether a cell index lies within grid bounds.

        @param grid_cell N-D integer coordinate (tuple[int,...]).
        @return True if inside bounds; False otherwise.
        """
        for i in range(self.dimension):
            if not (0 <= grid_cell[i] < self.grid.shape[i]):
                return False
        return True

    def dist(self, a, b) -> float:
        """
        @brief Euclidean distance between two N-D grid points.
        """
        return float(np.linalg.norm(np.subtract(a, b), ord=2))

    def adjacentCoordinates(self, node):
        """
        @brief Enumerate all adjacent coordinates in N-D including diagonals.

        @param node Reference grid cell (tuple[int,...]).
        @return List of neighbors without bounds or occupancy filtering.
        """
        offsets = [-1, 0, 1]
        combos = itertools.product(offsets, repeat=self.dimension)
        out = []
        for c in combos:
            if all(o == 0 for o in c):
                continue
            nb = tuple(node[i] + c[i] for i in range(self.dimension))
            out.append(nb)
        return out

    # ---------------------------
    # Collision checking on edges
    # ---------------------------

    def isEdgeFree(self, pt1, pt2) -> bool:
        """
        @brief Check if the straight segment between two points is collision-free.

        @details
        Uniformly samples @p self.pointSamples points on the segment [pt1, pt2],
        rounds each sample to the nearest grid index, and rejects the edge if any
        index is out-of-bounds or occupied.

        @param pt1 Start point (tuple[int,...]).
        @param pt2 End point   (tuple[int,...]).
        @return True if all sampled indices are valid and free.
        """
        for t in np.linspace(0.0, 1.0, self.pointSamples):
            point = (1 - t) * np.array(pt1, float) + t * np.array(pt2, float)
            idx = tuple(int(round(x)) for x in point)
            if not self.isValid(idx) or self.grid[idx] == 1:
                return False
        return True

    # ---------------------------
    # SDF-based safety terms
    # ---------------------------

    def EDFDeriv(self, node1, node2) -> float:
        """
        @brief Directional clearance change from node1 to node2.

        @details
        Approximates how clearance varies along the step. Lower values suggest
        moving into safer space. Used for neighbor ordering.
        """
        d = self.dist(node1, node2) + self.eps
        return (self.distanceTransform[node1] - self.distanceTransform[node2]) / d

    def EDFCost(self, node1, node2) -> float:
        """
        @brief Clearance penalty for traversing from node1 to node2.

        @details
        Uses average clearance along the step and scales the penalty with @p cw.
        Larger penalty discourages steps through narrow regions.
        """
        step_len = self.dist(node1, node2)
        avg_clearance = (self.distanceTransform[node1] + self.distanceTransform[node2]) * 0.5
        denom = (avg_clearance * step_len) + self.eps
        return self.cw / denom

    def _cost(self, a, b) -> float:
        """
        @brief Total step cost between two cells: geometric distance + safety penalty.
        """
        return self.dist(a, b) + self.EDFCost(a, b)

    # ---------------------------
    # Adaptive neighbor pruning
    # ---------------------------

    def getEDFNeigh(self, adjacentNodes, distVec):
        """
        @brief Keep up to @p maxNeigh neighbors best aligned with a target direction.

        @details
        Scores each neighbor by the cosine with @p distVec (goal direction blended
        with safest local direction), keeps the top @p maxNeigh.
        """
        node_np = np.array(self.node, float)
        scores = {}
        for nb in adjacentNodes:
            step = np.array(nb, float) - node_np
            n = np.linalg.norm(step) + 1e-9
            step /= n
            scores[nb] = float(np.dot(step, distVec))

        ranked = sorted(scores.items(), key=lambda kv: -kv[1])
        return [nb for nb, _ in ranked[: self.maxNeigh]]

    # ---------------------------
    # Main planning routine
    # ---------------------------

    def plan(self, start, goal, grid):
        """
        @brief Run FS-Planner on an N-D occupancy grid.

        @param start Start cell (tuple[int,...]).
        @param goal  Goal cell  (tuple[int,...]).
        @param grid  N-D occupancy array with values {0=free, 1=obstacle}.

        @return (success, path, info)
          - success: 1 on success, else 0
          - path   : list of N-D cells from start to goal (inclusive) or empty
          - info   : diagnostics list

        @diagnostics
          "Invalid start", "Invalid goal", "Start has obstacle",
          "Goal has obstacle", "Start and goal are same",
          "Failed to reconstruct path."
        """
        # Initialize problem
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.grid = grid
        self.path = []
        self.info = []
        self.success = 0

        self.dimension = len(start)
        self.sizes = [np.size(grid, axis=i) for i in range(self.dimension)]

        # Input checks
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

        # Precompute clearance map (EDT on free space)
        # Note: distance_transform_edt expects 1 for free space in our use.
        self.distanceTransform = distance_transform_edt(1 - self.grid)

        # A* state
        heap = []
        visited = np.zeros(self.grid.shape, bool)
        parents = {}
        g_score = np.full(self.grid.shape, np.inf, np.float32)

        g_score[self.start] = 0.0
        f0 = self.dist(self.start, self.goal)
        heapq.heappush(heap, (f0, 0.0, self.start))
        parents[self.start] = self.start
        self.node = self.start

        # Main loop (A* + Lazy Theta*)
        while heap:
            _, g_cur, self.node = heapq.heappop(heap)

            if self.node == self.goal:
                self.success = 1
                break

            if visited[self.node]:
                continue
            visited[self.node] = 1

            # Raw neighbors
            adjacentNodes = self.adjacentCoordinates(self.node)
            # Filter invalid/occupied/visited
            adjacentNodes = [
                nb for nb in adjacentNodes
                if self.isValid(nb) and self.grid[nb] == 0 and not visited[nb]
            ]

            # Direction blending: goal direction + safest local direction
            if adjacentNodes:
                goal_dir = np.array(self.goal, float) - np.array(self.node, float)
                goal_dir /= (np.linalg.norm(goal_dir) + 1e-9)

                # Pick neighbor with smallest clearance derivative as proxy for safer move
                sdf_deriv = {adj: self.EDFDeriv(self.node, adj) for adj in adjacentNodes}
                best_sdf_nb = min(sdf_deriv, key=sdf_deriv.get)
                sdf_dir = np.array(best_sdf_nb, float) - np.array(self.node, float)
                sdf_dir /= (np.linalg.norm(sdf_dir) + 1e-9)

                # Blend and prune
                distVec = goal_dir + sdf_dir
                distVec /= (np.linalg.norm(distVec) + 1e-9)
                adjacentNodes = self.getEDFNeigh(adjacentNodes, distVec)

            # Lazy Theta*: attempt to connect via parent if possible
            pu = parents.get(self.node, self.node)

            for nb in adjacentNodes:
                # Try LOS from parent's parent to nb (shortcut)
                if self.isEdgeFree(pu, nb):
                    g_cand = g_score[pu] + self._cost(pu, nb)
                    parent_cand = pu
                else:
                    g_cand = g_score[self.node] + self._cost(self.node, nb)
                    parent_cand = self.node

                if g_cand < g_score[nb]:
                    g_score[nb] = g_cand
                    parents[nb] = parent_cand
                    f = g_cand + self.dist(nb, self.goal)  # Euclidean heuristic
                    heapq.heappush(heap, (f, g_cand, nb))

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
