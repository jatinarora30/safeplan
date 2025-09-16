"""
@file weighted_astar.py
@brief Weighted A* planner for N-dimensional occupancy grids.

@details
Implements Weighted A* on an N-D NumPy occupancy grid where value 0 denotes free
space and value 1 denotes an obstacle. The planner accepts start and goal grid
indices and returns a collision-free path if one exists. Weighted A* scales the
heuristic by a user-specified weight to favor greedy expansions and reduce
runtime at the cost of potential suboptimality when the weight exceeds 1.

This implementation expands a full 3^N - 1 neighborhood (axis-aligned and
diagonal moves). Axis-aligned steps use unit cost; off-axis steps use the
Euclidean length of the step vector. A priority queue orders expansions by
g(n) + w·h(n), where g(n) is the accumulated path cost and h(n) is the chosen
heuristic (Manhattan in index space).

Neighborhood generation is provided by @ref adjacentCoordinates().

@par Constructor Arguments
- @p weight : float — multiplier applied to the heuristic (w ≥ 0).
              - w = 1 behaves like standard A* with this heuristic.
              - w > 1 is greedier and usually faster but may yield a longer path.
              - w < 1 is more conservative and may expand more nodes.

@par Inputs (to @ref plan())
- @p start : tuple[int, ...] — start grid cell (for example, (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}

@par Outputs
- @p success : int — 1 if a path is found, else 0
- @p path    : list[tuple[int, ...]] — sequence of cells from start to goal (inclusive)
- @p info    : list[str] — diagnostics (for example, "Invalid goal")

@note
- Coordinates are handled in grid index space.
- Step costs: axis steps cost 1.0; diagonal/off-axis steps use the Euclidean
  length of the index step.
- The neighborhood includes all immediate offsets except the zero vector.
- Heuristic used here is Manhattan distance; adjust if you change the move set.
- With w > 1, the solution is not guaranteed to be optimal.

@see BasePlanner

@references
- Pohl-style Weighted A* discussion and analysis:
  https://www.sciencedirect.com/science/article/pii/S000437020900068X
"""


from .baseplanner import BasePlanner
import heapq
import itertools
import numpy as np

class WeightedAStar(BasePlanner):
    
    def __init__(self,weight):
        """
        @brief Construct the Weighted A* planner.
        @param weight float Heuristic multiplier (w ≥ 0).
        @post Instance is initialized; outputs are cleared.
        """
        
        self.success=0
        self.weight=weight
        self.info=[]
        
        self.path= []
     
        
    def isValid(self, grid_cell):
        """
        @brief Check whether a cell lies within grid bounds.
        @param grid_cell tuple[int, ...] Cell index to validate.
        @return bool True if inside grid extent; otherwise False.
        """
        for i in range(self.dimension):
            if not (0 <= grid_cell[i] < self.grid.shape[i]):
                return False
        return True

    def heuristics(self,node1,node2):
        """
        @brief Heuristic distance between two nodes (Manhattan).
        @param node1 tuple[int, ...] First node.
        @param node2 tuple[int, ...] Second node.
        @return float Sum of absolute coordinate differences.
        @note Choose a heuristic consistent with the allowed moves and step costs.
        """
        cost=0
        for i in range(0,self.dimension):
            cost+=abs(node1[i]-node2[i])
        return cost
            
            
    def adjacentCoordinates(self,node):
        """
        @brief Enumerate all adjacent coordinates in N-D (including diagonals).
        @details
        Generates all offset combinations in {-1, 0, 1} per dimension, excluding
        the all-zero offset. Validity and obstacle checks are performed during
        expansion in @ref plan().
        @param node tuple[int, ...] The reference grid cell.
        @return list<tuple[int, ...]> Neighbor coordinates (not yet filtered).
        """
        offsets=[-1,0,1]
        combinations=itertools.product(offsets,repeat=self.dimension)
        adjacentNodes=[]
        for c in combinations:
            if all (o ==0 for o in c):
                continue
            nb= tuple(node[i]+c[i] for i in range(self.dimension))
            adjacentNodes.append(nb)
        return adjacentNodes
                
        
    def plan(self,start,goal,grid):
        """
        @brief Run Weighted A* from start to goal on the provided grid.
        @param start tuple[int, ...] Start index in the N-D grid.
        @param goal  tuple[int, ...] Goal index in the N-D grid.
        @param grid  numpy.ndarray Occupancy grid (0=free, 1=obstacle).
        @return tuple
                - @c success (int) : 1 if a path is found, else 0
                - @c path (list[tuple[int, ...]]) : sequence from start to goal (inclusive)
                - @c info (list[str]) : diagnostics or failure reasons
        @par Diagnostics
        - "Invalid start", "Invalid  goal"
        - "Start has obstacle", "Goal has obstacle"
        - "Start and goal are same"
        - "Failed to reconstruct path."
        @note Uses priority g + w·h. With weights greater than 1 the algorithm
              is greedier and may return a suboptimal path more quickly.
        """
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
        f0=self.heuristics(self.start,self.goal)
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
        
            
            #calculating alternate nodes
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
                            total_cost=self.weight*self.heuristics(adjacentNodes[k],self.goal)+g_updated
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
