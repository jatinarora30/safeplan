"""
@file dijkstra.py
@brief Dijkstra planner for N-dimensional occupancy grids.

@details
Implements Dijkstra’s shortest-path search on an N-D NumPy occupancy grid where
value 0 denotes free space and value 1 denotes an obstacle. The planner accepts
start and goal grid indices and returns a collision-free path with minimum
accumulated cost if one exists.

This implementation expands a full 3^N - 1 neighborhood (axis-aligned and diagonal
moves). Axis-aligned steps use unit cost; diagonal or off-axis steps use the
Euclidean length of the step vector. A priority queue orders expansions by the
current best distance from the start. No heuristic is used.

Neighborhood generation is provided by @ref adjacentCoordinates().

@par Inputs
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
  length of the step vector based on index differences.
- The neighborhood includes all immediate offsets except the zero vector.

@see BasePlanner

@references
- Dijkstra, E. W., “A note on two problems in connexion with graphs,” 1959.
  Overview link: https://www.semanticscholar.org/paper/A-note-on-two-problems-in-connexion-with-graphs-Dijkstra/45786063578e814444b8247028970758bbbd0488
"""

from .baseplanner import BasePlanner
import heapq
import itertools
import numpy as np

class Dijkstra(BasePlanner):
    
    def __init__(self):
        """
        @brief Construct the Dijkstra planner.

        @post Instance is initialized; outputs are cleared.
        """
        
        self.success=0
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
        @brief Run Dijkstra planning from @p start to @p goal on @p grid.
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
        @note Uses a priority queue keyed by distance from the start. No heuristic is used.
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
        heapq.heappush(heap,(0,self.start))
        self.parent=None
        self.node=self.start
        
        while heap:
            
            g,self.node=heapq.heappop(heap)
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
                            total_cost=g_updated
                            heapq.heappush(heap,(g_updated,adjacentNodes[k]))
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
