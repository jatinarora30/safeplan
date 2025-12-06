"""
@file upp.py
@brief Universal Path Planner (UPP) with direction-based adaptive safety.

@details
This planner blends geometric distance and safety potential for N-D occupancy grids.
It adaptively adjusts the safety weighting β based on *signed progress* toward the goal.

Key ideas:
- Combined distance heuristic: mix of Manhattan and Chebyshev distances.
- Safety potential: convolution-based obstacle cost field.
- Adaptive β: increases when the planner moves closer to the goal, decreases when moving away or stalling.
- Priority queue A* search accumulating total path cost.

@see BasePlanner
"""

from .baseplanner import BasePlanner
import heapq
import itertools
import numpy as np
from scipy.signal import fftconvolve
from scipy.ndimage import distance_transform_edt


class UPP(BasePlanner):
    """
    Universal Path Planner blending distance and adaptive safety weighting (β),
    with windowed hysteresis-based adaptive α (turn-aware).
    """

    def __init__(self, alphaBase, betaBase, radiusBase, epsilon,
                 betaMin,betaMax,betaDecay,betaRecovery,betaPatience,goalTol,
                 alphaMin,alphaMax,alphaDecay,alphaRecovery,tolAngular,turnTarget,turnWindow,
                 radiusMin,radiusMax):
        
        self.alpha = alphaBase
        self.beta = betaBase
        self.R = radiusBase
        self.epsilon = epsilon
        
        self.betaMin = betaMin
        self.betaMax = betaMax
        self.decay = betaDecay
        self.recovery = betaRecovery
        self.patience = betaPatience
        self.tol = goalTol
        
        self.radiusMin=radiusMin
        self.radiusMax=radiusMax
        
        self.alphaMin = alphaMin
        self.alphaMax = alphaMax
        self.alpha_decay = alphaDecay    
        self.alpha_recovery = alphaRecovery  
        self.tolAngular = np.deg2rad(tolAngular)   
        self.turn_target = np.deg2rad(turnTarget)  
        self.turn_window_K = turnWindow            

        self.turn_sum = 0.0   
        self.turn_iter = 0               
        self.adaptive_beta=True
        self.adaptive_alpha=True
        self.success = 0
        self.info = []
        self.path = []

    # ---------- Utility Functions ----------
    def isValid(self, cell):
        for i in range(self.dimension):
            if not (0 <= cell[i] < self.grid.shape[i]):
                return False
        return True

    def manhattan(self, a, b):
        return sum(abs(a[i] - b[i]) for i in range(self.dimension))

    def chebyshev(self, a, b):
        return max(abs(x - y) for x, y in zip(a, b))

    def _l2(self, a, b):
        return float(np.sqrt(sum((a[i] - b[i]) ** 2 for i in range(self.dimension))))

    def adjacent(self, node, offsets=[-1, 0, 1]):
        for o in itertools.product(offsets, repeat=self.dimension):
            if all(v == 0 for v in o):
                continue
            yield tuple(node[i] + o[i] for i in range(self.dimension))

    # ---------- Safety Precomputation ----------
    def precomputeSafety(self):
     
        grid = self.grid
        obs = (grid == 1).astype(np.float32)
        R = int(self.R)

        # If no obstacles or invalid radius → no safety
        if R <= 0 or not np.any(obs):
            zero = np.zeros_like(obs, dtype=np.float32)
            self.preSafety = zero
            return

        nd = obs.ndim  
        grids = np.ogrid[tuple(slice(-R, R + 1) for _ in range(nd))]
        from functools import reduce
        d_inf = reduce(np.maximum, (np.abs(g) for g in grids)).astype(np.float32)

        K_inv = np.zeros_like(d_inf, dtype=np.float32)
        mask_inv = (d_inf > 0) & (d_inf <= R)
        K_inv[mask_inv] = 1.0 / (d_inf[mask_inv] + self.epsilon)

        S_sumInv = fftconvolve(obs, K_inv, mode="same").astype(np.float32)

        free = (grid == 0)
        D = self.D

        preB = np.zeros_like(D, dtype=np.float32)
        maskB = free & (S_sumInv > 0)
        preB[maskB] = S_sumInv[maskB]
        self.preSafety = preB.astype(np.float32)


    
    def heuristic(self, n1, n2):
        base = self.alpha * self.manhattan(n1, n2) + (1 - self.alpha) * self.chebyshev(n1, n2)
        return base + self.beta * self.preSafety[n1]

    # ---------- Main Planner ----------
    def plan(self, start, goal, grid):
        """
        Plan a path from start to goal on the given occupancy grid.
        adaptive_beta : bool
            If True, enables direction-aware adaptive β updates.
            If False, β remains constant throughout planning.
        """
        self.start, self.goal, self.grid = tuple(start), tuple(goal), grid
        self.path, self.info = [], []
        self.success, self.dimension = 0, len(start)

        # reset α accumulators each plan call
        self.turn_sum = 0.0
        self.turn_iter = 0

        prev_dist, stalled, iteration = None, 0, 0

        # Validity checks
        if not self.isValid(self.start):
            return 0, [], ["Invalid start"]
        if not self.isValid(self.goal):
            return 0, [], ["Invalid goal"]
        if grid[self.goal] == 1:
            return 0, [], ["Goal occupied"]
        if grid[self.start] == 1:
            return 0, [], ["Start occupied"]
        if self.start == self.goal:
            return 1, [self.start], ["Start and goal are the same"]

        # Environment-driven initial scaling (unchanged)
        self.D = distance_transform_edt(grid == 0).astype(np.float32)
        free = (grid == 0)
        rho = float(np.count_nonzero(grid == 1)) / float(grid.size)
        if np.any(free):
            mu, sigma = float(np.mean(self.D[free])), float(np.std(self.D[free]))
            beta_raw = self.beta * rho * (sigma / (mu + self.epsilon))
            self.beta = float(np.clip(beta_raw, self.betaMin, self.betaMax))
            R_new = int(round(self.R * (mu + sigma)))
            self.R = int(np.clip(R_new, self.radiusMin,self.radiusMax))

        self.precomputeSafety()

        open_set, visited, parents = [], np.zeros(grid.shape, bool), {}
        g_score = np.full(grid.shape, np.inf, np.float32)
        g_score[self.start] = 0.0

        heapq.heappush(open_set, (self.heuristic(self.start, self.goal), 0.0, self.start))

        while open_set:
            iteration += 1
            _, g, current = heapq.heappop(open_set)

            cur_dist = self._l2(current, self.goal)
            if prev_dist is None:
                prev_dist = cur_dist
            delta = cur_dist - prev_dist  # signed progress

            # --- Direction-aware β adaptation (unchanged) ---
            if self.adaptive_beta:
                if delta < -self.tol:
                    stalled = 0
                    new_beta = min(self.beta * self.recovery, self.betaMax)
                    if new_beta > self.beta:
                        self.beta = new_beta
                elif delta > self.tol :
                        stalled = 0
                        new_beta = max(self.beta * self.decay, self.betaMin)
                        if new_beta < self.beta:
                            self.beta = new_beta
                elif delta<self.tol:
                    stalled += 1
                    if stalled >= self.patience:
                        new_beta = max(self.beta * self.decay, self.betaMin)
                        if new_beta < self.beta:
                            self.beta = new_beta
                        stalled = 0

            prev_dist = cur_dist

            if self.adaptive_alpha:
                if current in parents:
                    prev_node = parents[current]
                    move_vec = np.array(current) - np.array(prev_node)
                    goal_vec = np.array(self.goal) - np.array(current)
                    m_norm = np.linalg.norm(move_vec)
                    g_norm = np.linalg.norm(goal_vec)

                    if m_norm > 1e-6 and g_norm > 1e-6:
                        move_ang = np.arctan2(move_vec[1], move_vec[0])
                        goal_ang = np.arctan2(goal_vec[1], goal_vec[0])
                        # smallest signed angle difference in [-pi, pi]
                        raw_angle = (goal_ang - move_ang + np.pi) % (2 * np.pi) - np.pi
                        turn_angle = abs(raw_angle)  # magnitude of turn needed
                    else:
                        turn_angle = 0.0
                else:
                    turn_angle = 0.0

                signed_turn = turn_angle - self.turn_target
                self.turn_sum += signed_turn
                self.turn_iter += 1

                # Every K iterations, apply hysteresis & reset
                if self.turn_iter >= self.turn_window_K:
                    if self.turn_sum > self.tolAngular:
                        # Too much turning over the window -> α ↑ (straighter paths)
                        self.alpha = min(self.alpha * self.alpha_recovery, self.alphaMax)
                    elif self.turn_sum < -self.tolAngular:
                        # Consistently low turning -> α ↓ (more diagonal freedom)
                        self.alpha = max(self.alpha * self.alpha_decay, self.alphaMin)

                    # Reset window
                    self.turn_sum = 0.0
                    self.turn_iter = 0
            # -------------------------------------------------

            if current == self.goal:
                self.success = 1
                break

            if visited[current]:
                continue
            visited[current] = True

            for nb in self.adjacent(current):
                if not self.isValid(nb) or grid[nb] == 1 or visited[nb]:
                    continue
                delta_vec = tuple(nb[i] - current[i] for i in range(self.dimension))
                is_axis = sum(abs(d) for d in delta_vec) == 1
                step = 1.0 if is_axis else float(np.linalg.norm(delta_vec, ord=2))
                g_new = g + step
                if g_new < g_score[nb]:
                    g_score[nb] = g_new
                    parents[nb] = current
                    f_nb = g_new + self.heuristic(nb, self.goal)
                    heapq.heappush(open_set, (f_nb, g_new, nb))

        # -------- Path Reconstruction --------
        if self.success:
            node = self.goal
            while node != self.start:
                self.path.append(node)
                node = parents.get(node)
                if node is None:
                    return 0, [], ["Failed to reconstruct path"]
            self.path.append(self.start)
            self.path.reverse()

        return self.success, self.path, self.info
