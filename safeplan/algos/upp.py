

from .baseplanner import BasePlanner
import heapq
import itertools
import numpy as np

class UPP(BasePlanner):

    def __init__(self,alpha,beta,radius,epsilon):

        self.alpha=alpha
        self.beta=beta
        self.R=radius
        self.epsilon=epsilon

        self.success=0
        self.info=[]

        self.path= []

    def isValid(self, grid_cell):
        
        for i in range(self.dimension):
            if not (0 <= grid_cell[i] < self.grid.shape[i]):
                return False
        return True

    def manhattanHeuristics(self,node1,node2):
        r"""
        @brief Manhattan (L1) distance heuristic.

        @param node1 tuple[int, ...]
               First grid index.
        @param node2 tuple[int, ...]
               Second grid index.

        @return float
                \f$\sum_i |node1_i - node2_i|\f$.
        """
        cost=0
        for i in range(0,self.dimension):
            cost+=abs(node1[i]-node2[i])
        return cost

    def chebyshevHeuristics(self, node1, node2):
        r"""
        @brief Chebyshev (L∞) distance heuristic.

        @param node1 tuple[int, ...]
        @param node2 tuple[int, ...]
        @return float
                \f$\max_i |node1_i - node2_i|\f$.
        """
        return max(abs(a-b) for a, b in zip(node1, node2))

    def adjacentCoordinates(self,node,offsets=[-1,0,1]):
        r"""
        @brief Generate adjacent coordinates around a node.

        @details
        Produces all neighbors in the 3^N - 1 hypercube around @p node, i.e.,
        offsets in {-1,0,+1}^N excluding the all-zero offset. Validity and
        occupancy checks are performed by the caller.

        @param node tuple[int, ...]
               Center grid index whose neighbors are requested.
        @param offsets list[int]
               Set of per-axis offsets (default: [-1, 0, 1]).

        @return list[tuple[int, ...]]
                Neighbor coordinates (not filtered by bounds or occupancy).
        """
        combinations=itertools.product(offsets,repeat=self.dimension)
        adjacentNodes=[]
        for c in combinations:
            if all (o ==0 for o in c):
                continue
            nb= tuple(node[i]+c[i] for i in range(self.dimension))
            adjacentNodes.append(nb)
        return adjacentNodes

    def precomputeSafety(self):
        r"""
        @brief Build the safety potential map via FFT convolution.


        @post `self.preSafety` is allocated with the same shape as `self.grid`.
        """
        """
        Compute safety = sum over obstacles within Chebyshev radius R of 1/(d_inf + epsilon)
        using FFT convolution. Stores float32 map in self.preSafety with same shape as self.grid.
        """

        import numpy as np                              # NumPy for array ops
        from scipy.signal import fftconvolve            # Fast N-D convolution via FFT

        obs = (self.grid == 1).astype(np.float32)       # Binary obstacle mask: 1 where grid has obstacles; cast to float32
        R = int(self.R)                                  # Ensure radius is an int (kernel size depends on R)
        if R <= 0 or not np.any(obs):                    # Early exit: no radius or no obstacles -> no safety influence
            self.preSafety = np.zeros_like(obs, dtype=np.float32)
            return

        nd = obs.ndim                                   # Number of dimensions in the grid (2D, 3D, …)

        # --- Build Chebyshev-distance kernel K of shape (2R+1, ..., 2R+1) ---
        grids = np.ogrid[tuple(slice(-R, R + 1) for _ in range(nd))]
        # ^ Create nd open grids spanning -R..R along each axis with broadcasting shapes

        from functools import reduce
        d_inf = reduce(np.maximum, (np.abs(g) for g in grids))

        # ^ Chebyshev (L∞) distance at each kernel offset: max over axes of |offset|

        K = np.zeros_like(d_inf, dtype=np.float32)      # Allocate the kernel array (same shape as d_inf)
        mask = (d_inf > 0) & (d_inf <= R)               # Include offsets strictly within radius; exclude center (d=0)

        K[mask] = 1.0 / (d_inf[mask] + float(self.epsilon))
        # ^ Inverse-distance weight for every offset in radius: 1/(d_inf + ε); ε avoids divide-by-zero

        # --- Convolve obstacle map with kernel (zero-padded boundaries) ---
        S = fftconvolve(obs, K, mode="same")
        # ^ Linear convolution in frequency domain; "same" crops to the original grid size

        S = np.maximum(S, 0).astype(np.float32)         # Clamp tiny negative round-off to 0; store as float32
        self.preSafety = S                               # Save the safety influence map for use in planning

    def combinedHeuristics(self,node1,node2):
        optimalCost=self.alpha*self.manhattanHeuristics(node1,node2)+(1-self.alpha)*self.chebyshevHeuristics(node1,node2)
        safetyCost=self.preSafety[node1]
        return optimalCost+self.beta*safetyCost

    def plan(self,start,goal,grid):
        self.start=tuple(start)
        self.goal=tuple(goal)
        self.grid=grid
        self.path =[]
        self.info = []
        self.success=0
        self.dimension=len(start)
        self.sizes=[]
        for i in range(0,self.dimension):
            self.sizes.append(np.size(grid,axis=i))

        if not self.isValid(self.start):
            self.info.append("Invalid start ")
            return self.success,self.path,self.info

        if not self.isValid(self.goal):
            self.info.append("Invalid  goal")
            return self.success,self.path,self.info

        if self.grid[self.goal]==1:
            self.info.append("Goal has obstacle")
            return self.success,self.path,self.info

        if self.grid[self.start]==1:
            self.info.append("Start has obstacle")
            return self.success,self.path,self.info

        if self.start==self.goal:
            self.info.append("Start and goal are same")
            self.success = 1
            self.path = [self.start]
            return self.success,self.path,self.info

        heap=[]
        visited=np.zeros(self.grid.shape,bool)
        parents={}
        g_score = np.full(self.grid.shape, np.inf, np.float32)
        g_score[self.start]=0
        self.precomputeSafety()
        f0=self.combinedHeuristics(self.start,self.goal)
        heapq.heappush(heap,(f0,0,self.start))
        self.parent=None
        self.node=self.start

        while heap:
            _,g,self.node=heapq.heappop(heap)
            if self.node==self.goal:
                break

            if visited[self.node]:
                continue

            visited[self.node]=1

            # calculating alternate nodes
            adjacentNodes=self.adjacentCoordinates(self.node)

            for k in range(len(adjacentNodes)):
                if self.isValid(adjacentNodes[k]) and self.grid[adjacentNodes[k]]==0 and visited[adjacentNodes[k]]==0:
                    self.parent=self.node
                    if adjacentNodes[k]==self.goal:
                        parents[adjacentNodes[k]]=self.parent
                        self.success=1
                        break
                    else:
                        delta = tuple(adjacentNodes[k][i] - self.node[i] for i in range(self.dimension))
                        is_axis_step = sum(abs(d) for d in delta) == 1
                        step_cost = 1.0 if is_axis_step else float(np.linalg.norm(delta, ord=2))  # or Chebyshev via max(abs(d))
                        g_updated = g + step_cost

                        if g_updated < g_score[adjacentNodes[k]]:
                            g_score[adjacentNodes[k]] = g_updated
                            total_cost=self.combinedHeuristics(adjacentNodes[k],self.goal)+g_updated
                            heapq.heappush(heap,(total_cost,g_updated,adjacentNodes[k]))
                            parents[adjacentNodes[k]]=self.parent
            if self.success==1:
                break

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

        return self.success,self.path,self.info
