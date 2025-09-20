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
  - Collision checking along edges via fast N-D voxel traversal (grid DDA / Bresenham).

FS-Planner trades strict shortest-path optimality for higher clearance and
smoother, lower-risk routes. Tuning @p cw adjusts this trade-off.

@par Inputs (to plan())
- @p start : tuple[int, ...] — start grid cell
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), occupancy {0=free, 1=obstacle}

@par Constructor Args
- @p pointSamples : int — (kept for API-compat; not used by the DDA LOS)
- @p cw           : float — clearance weight in edge cost
- @p epsilon      : float — small constant for numerical stability
- @p maxNeigh     : int — cap on neighbors kept after adaptive pruning

@par Outputs (from plan())
- @p success : int — 1 if a path is found, else 0
- @p path    : list[tuple[int, ...]] — start→goal (inclusive) or empty
- @p info    : list[str] — diagnostics (e.g., "Invalid goal")

@note
- Heuristic optimality guarantees do not strictly hold with clearance penalties.
- Set cw=0 to approximate baseline A* (still with LOS smoothing).
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
                            (Kept for API compatibility; LOS now uses integer stepping.)
        @param cw Clearance weight in the edge cost.
        @param epsilon Small constant to avoid division by zero and improve stability.
        @param maxNeigh Maximum number of neighbors retained after pruning.
        """
        self.success = 0
        self.pointSamples = int(pointSamples)  # not used in DDA, retained for API compatibility
        self.cw = float(cw)
        self.eps = float(epsilon)
        self.maxNeigh = int(maxNeigh)

        self.info = []
        self.path = []

        # Lazily filled per-plan()
        self._neighbor_offsets = None
        self._offset_index = None
        self._neighbor_unit = None
        self._neighbor_step_len = None
        self._los_cache = None  # caches (u,v)->bool results for LOS

    # ---------------------------
    # Basic helpers and geometry
    # ---------------------------

    def isValid(self, grid_cell) -> bool:
        """Check whether a cell index lies within grid bounds."""
        for i in range(self.dimension):
            v = grid_cell[i]
            if v < 0 or v >= self.grid.shape[i]:
                return False
        return True

    def dist(self, a, b) -> float:
        """Euclidean distance between two N-D grid points."""
        d = 0.0
        for i in range(self.dimension):
            dx = float(a[i] - b[i])
            d += dx * dx
        return float(d ** 0.5)

    def adjacentCoordinates(self, node):
        """Enumerate all adjacent coordinates in N-D including diagonals."""
        out = []
        # Use prebuilt offsets
        for off in self._neighbor_offsets:
            out.append(tuple(node[i] + off[i] for i in range(self.dimension)))
        return out

    # ---------------------------
    # Collision checking on edges
    # ---------------------------

    def _los_key(self, a, b):
        # Order-independent cache key
        return (a, b) if a <= b else (b, a)

    def _isEdgeFree2D(self, pt1, pt2, grid) -> bool:
        """
        Fast 2-D supercover Bresenham line-of-sight.
        Visits all cells a straight segment crosses.
        """
        (x0, y0) = pt1
        (x1, y1) = pt2

        # Basic validity & trivial path
        if not (self.isValid(pt1) and self.isValid(pt2)):
            return False
        if (x0, y0) == (x1, y1):
            return grid[x0, y0] == 0

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if grid[x0, y0] == 1:
                return False
            if x0 == x1 and y0 == y1:
                break
            e2 = err << 1  # 2*err
            moved_x = moved_y = False
            if e2 > -dy:
                err -= dy
                x0 += sx
                moved_x = True
            if e2 < dx:
                err += dx
                y0 += sy
                moved_y = True
            # supercover corner check
            if moved_x and moved_y:
                if grid[x0 - sx, y0] == 1 or grid[x0, y0 - sy] == 1:
                    return False
        return True

    def isEdgeFree(self, pt1, pt2) -> bool:
        """
        Check if the straight segment between two points is collision-free.
        Uses supercover Bresenham in 2-D; falls back to simple voxel stepping for N-D.
        """
        key = self._los_key(pt1, pt2)
        hit = self._los_cache.get(key)
        if hit is not None:
            return hit

        grid = self.grid
        if self.dimension == 2:
            ok = self._isEdgeFree2D(pt1, pt2, grid)
            self._los_cache[key] = ok
            return ok

        # Fallback N-D (cheap integer stepping visiting roughly the segment corridor)
        a = tuple(int(x) for x in pt1)
        b = tuple(int(x) for x in pt2)
        if not (self.isValid(a) and self.isValid(b)):
            self._los_cache[key] = False
            return False

        step = tuple(1 if (bi - ai) > 0 else (-1 if (bi - ai) < 0 else 0) for ai, bi in zip(a, b))
        cell = list(a)
        if grid[tuple(cell)] == 1:
            self._los_cache[key] = False
            return False

        while tuple(cell) != b:
            # advance along axis with largest remaining difference
            diffs = [abs(b[i] - cell[i]) for i in range(self.dimension)]
            k = int(np.argmax(diffs))
            cell[k] += step[k]
            cur = tuple(cell)
            if (not self.isValid(cur)) or grid[cur] == 1:
                self._los_cache[key] = False
                return False

        self._los_cache[key] = True
        return True

    # ---------------------------
    # SDF-based safety terms
    # ---------------------------

    def EDFDeriv(self, node1, node2) -> float:
        """Directional clearance change from node1 to node2."""
        d = self.dist(node1, node2) + self.eps
        return (float(self.distanceTransform[node1]) - float(self.distanceTransform[node2])) / d

    def EDFCost(self, node1, node2) -> float:
        """Clearance penalty for traversing from node1 to node2."""
        step_len = self.dist(node1, node2)
        c1 = float(self.distanceTransform[node1])
        c2 = float(self.distanceTransform[node2])
        avg_clearance = (c1 + c2) * 0.5
        denom = (avg_clearance * step_len) + self.eps
        return self.cw / denom

    def _cost(self, a, b) -> float:
        """Total step cost between two cells: geometric distance + safety penalty."""
        return self.dist(a, b) + self.EDFCost(a, b)

    # ---------------------------
    # Adaptive neighbor pruning
    # ---------------------------

    def getEDFNeigh(self, adjacentNodes, distVec):
        """
        Keep up to @p maxNeigh neighbors best aligned with a target direction.
        Uses precomputed neighbor unit vectors to avoid per-step allocations.
        """
        node_i = self.node
        ux = distVec  # already normalized (or near)
        best = []

        # Local refs for speed
        offset_index = self._offset_index
        neighbor_unit = self._neighbor_unit
        dim = self.dimension

        ux0 = ux[0]
        ux1 = ux[1] if dim > 1 else 0.0
        ux2 = ux[2] if dim > 2 else 0.0

        for nb in adjacentNodes:
            off = tuple(nb[i] - node_i[i] for i in range(dim))
            ui = offset_index.get(off)
            if ui is None:
                # Shouldn't happen, but be robust
                stepv = np.array(off, dtype=np.float32)
                n = float(np.linalg.norm(stepv)) + 1e-9
                stepv /= n
                score = float(stepv[0]) * ux0 + (float(stepv[1]) * ux1 if dim > 1 else 0.0) + (float(stepv[2]) * ux2 if dim > 2 else 0.0)
            else:
                vu = neighbor_unit[ui]
                score = vu[0] * ux0 + (vu[1] * ux1 if dim > 1 else 0.0) + (vu[2] * ux2 if dim > 2 else 0.0)
            best.append((score, nb))

        best.sort(key=lambda kv: -kv[0])
        k = self.maxNeigh
        return [nb for _, nb in best[:k]]

    # ---------------------------
    # Main planning routine
    # ---------------------------

    def plan(self, start, goal, grid):
        """
        Run FS-Planner on an N-D occupancy grid.

        @return (success, path, info)
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

        # Prebuild neighbor offsets (3^N - 1)
        offsets = (-1, 0, 1)
        combos = itertools.product(offsets, repeat=self.dimension)
        self._neighbor_offsets = [tuple(c) for c in combos if not all(o == 0 for o in c)]

        # Build fast lookup for offsets, precompute unit vectors & step lengths
        self._offset_index = {off: i for i, off in enumerate(self._neighbor_offsets)}
        unit = []
        step_len = []
        for off in self._neighbor_offsets:
            v0 = float(off[0])
            v1 = float(off[1]) if self.dimension > 1 else 0.0
            v2 = float(off[2]) if self.dimension > 2 else 0.0
            n = (v0 * v0 + v1 * v1 + v2 * v2) ** 0.5
            if n == 0.0:
                unit.append((0.0, 0.0, 0.0))
                step_len.append(0.0)
            else:
                unit.append((v0 / n, v1 / n, v2 / n))
                step_len.append(n)
        self._neighbor_unit = tuple(unit)
        self._neighbor_step_len = tuple(step_len)

        # LOS cache per-search
        self._los_cache = {}

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
        self.distanceTransform = distance_transform_edt(1 - self.grid).astype(np.float32)

        # Early out if direct LOS start->goal
        if self.isEdgeFree(self.start, self.goal):
            self.success = 1
            self.path = [self.start, self.goal]
            return self.success, self.path, self.info

        # A* state (smaller dtypes for speed)
        heap = []  # elements: (f, g, node)
        visited = np.zeros(self.grid.shape, np.uint8)
        parents = {}
        g_score = np.full(self.grid.shape, np.inf, np.float32)

        g_score[self.start] = 0.0
        f0 = self.dist(self.start, self.goal)
        heapq.heappush(heap, (f0, 0.0, self.start))
        parents[self.start] = self.start
        self.node = self.start

        # Local refs for speed
        goal = self.goal
        grid = self.grid
        DT = self.distanceTransform
        cw = self.cw
        eps = self.eps
        dim = self.dimension
        dist_fn = self.dist
        is_edge_free = self.isEdgeFree

        # Main loop (A* + Lazy Theta*)
        while heap:
            _, g_cur, self.node = heapq.heappop(heap)

            if self.node == goal:
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
                if self.isValid(nb) and grid[nb] == 0 and not visited[nb]
            ]

            # Direction blending: goal direction + safest local direction
            if adjacentNodes:
                # goal_dir
                goal_dir = np.empty(dim, dtype=np.float32)
                for i in range(dim):
                    goal_dir[i] = float(goal[i] - self.node[i])
                gnorm = float(np.linalg.norm(goal_dir)) + 1e-9
                goal_dir /= gnorm

                # safest neighbor (min EDFDeriv)
                best_nb = None
                best_val = float("inf")
                for adj in adjacentNodes:
                    d = dist_fn(self.node, adj) + eps
                    val = (float(DT[self.node]) - float(DT[adj])) / d
                    if val < best_val:
                        best_val = val
                        best_nb = adj

                sdf_dir = np.empty(dim, dtype=np.float32)
                for i in range(dim):
                    sdf_dir[i] = float(best_nb[i] - self.node[i])
                snorm = float(np.linalg.norm(sdf_dir)) + 1e-9
                sdf_dir /= snorm

                # Blend (unit-ish) and prune
                distVec = goal_dir + sdf_dir
                dnorm = float(np.linalg.norm(distVec)) + 1e-9
                distVec /= dnorm
                adjacentNodes = self.getEDFNeigh(adjacentNodes, distVec)

            # Lazy Theta*: attempt to connect via parent if it could help
            pu = parents.get(self.node, self.node)

            for nb in adjacentNodes:
                use_parent = False
                if pu != self.node:
                    # Only try grandparent LOS if geometric distance can improve
                    if dist_fn(pu, nb) + 1e-9 < dist_fn(self.node, nb):
                        if is_edge_free(pu, nb):
                            use_parent = True

                if use_parent:
                    # cost via parent
                    step_len = dist_fn(pu, nb)
                    c1 = float(DT[pu])
                    c2 = float(DT[nb])
                    avg_clearance = (c1 + c2) * 0.5
                    g_cand = float(g_score[pu]) + step_len + (cw / (avg_clearance * step_len + eps))
                    parent_cand = pu
                else:
                    step_len = dist_fn(self.node, nb)
                    c1 = float(DT[self.node])
                    c2 = float(DT[nb])
                    avg_clearance = (c1 + c2) * 0.5
                    g_cand = float(g_score[self.node]) + step_len + (cw / (avg_clearance * step_len + eps))
                    parent_cand = self.node

                if g_cand < float(g_score[nb]):
                    g_score[nb] = g_cand
                    parents[nb] = parent_cand
                    f = g_cand + dist_fn(nb, goal)  # Euclidean heuristic
                    heapq.heappush(heap, (f, g_cand, nb))

        # Reconstruct path
        if self.success == 1:
            path_node = goal
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
