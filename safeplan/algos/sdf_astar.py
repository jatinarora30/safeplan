"""
@file sdf_astar.py
@brief Signed Distance Field (SDF) guided A* planner for N-dimensional occupancy grids.

@details
Implements an A* variant that uses a Signed Distance Field (SDF) to bias search
toward safer regions. The environment is an N-D NumPy occupancy grid where value
0 denotes free space and value 1 denotes an obstacle. The planner accepts start
and goal grid indices and returns a collision-free path if one exists.

Key ideas:
- A standard grid A* framework: priority queue, cumulative path cost, and a
  neighborhood that includes all immediate offsets.
- An SDF-based safety term: a distance transform and its gradient encourage
  exploration toward areas with better clearance (see @ref safetyHeuristics()).
- A combined priority: mixes travel distance, path cost so far, and a local
  safety measure based on the SDF gradient along the current motion direction
  (see @ref combinedHeuristics()).

Heuristic and neighborhood generation are provided by @ref heuristics() and
@ref adjacentCoordinates(). Ensure the heuristic is compatible with the allowed
moves and step costs.

@par Constructor Arguments
- @p k1 : float — weight on path-cost term inside the combined priority
- @p k2 : float — weight on heuristic distance term inside the combined priority

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
- The SDF is computed once per plan call using a distance transform on the free
  space mask; its gradient is reused in the safety term.
- The neighborhood includes all immediate offsets except the zero vector.
- The safety term normalizes the SDF gradient change along the current motion
  direction, with simple clamping to avoid division by zero.

@see BasePlanner

@references
- SDF-guided A* : https://ieeexplore.ieee.org/document/10364842/
"""


from .baseplanner import BasePlanner
import heapq
from scipy.ndimage import distance_transform_edt
import itertools
import numpy as np

class SDFAStar(BasePlanner):
    
    def __init__(self,k1,k2):
        """
        @brief Construct the SDF-guided A* planner.
        @param k1 float Weight on the path-cost term in the combined priority.
        @param k2 float Weight on the distance-to-goal term in the combined priority.
        @post Instance is initialized; outputs are cleared.
        """
        
        self.success=0
        self.k1=k1
        self.k2=k2
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
        @note Choose a heuristic that matches the allowed moves and step costs.
        """
        cost=0
        for i in range(0,self.dimension):
            cost+=abs(node1[i]-node2[i])
        return cost

    def safetyHeuristics(self,node,parent):
        """
        @brief Local SDF-based safety measure at a node, relative to the motion direction.
        @details
        Uses the precomputed SDF (@c self.distanceTransform) and its gradient
        (@c self.grad). If the SDF at the node is negative, the location is inside
        an obstacle and the safety measure is set to an infinite penalty.

        Otherwise, the method estimates how the SDF changes in the direction of
        travel from the parent to the node. This directional change is normalized
        by the largest absolute gradient component at that location to keep the
        value in a comparable range.

        @param node tuple[int, ...] Node at which to measure safety.
        @param parent tuple[int, ...] Parent node used to determine motion direction.
        @return float A nonnegative safety value; larger values indicate lower safety.
        """
        
        idx = tuple(np.rint(node).astype(int))
        
        if self.distanceTransform[idx]<0:
            return np.inf
        
        if parent is not None:
            steer=np.array(node)-np.array(parent)
            steerDir=steer/(np.linalg.norm(steer)+1e-9)
        else:
            steerDir=1
            
        grad = np.array([self.grad[i][idx] for i in range(self.dimension)], dtype=float)

        directionalDerivative = np.dot(grad, steerDir)
        maxDerivative = np.max(np.abs(grad)) + 1e-9  
        normalizedDerivative = abs(directionalDerivative) / maxDerivative
        
        
        
        return normalizedDerivative

    def combinedHeuristics(self,node,parent,pathCost,goal):
        """
        @brief Combined priority for A* that blends distance and SDF-based safety.
        @details
        The combined value increases with path cost so far, with distance to goal,
        and with the local safety measure from @ref safetyHeuristics(). The weights
        @p k1 and @p k2 scale the influence of path cost and distance.
        @param node tuple[int, ...] Current node.
        @param parent tuple[int, ...] Parent of the current node (for motion direction).
        @param pathCost float Cumulative path cost @c g so far.
        @param goal tuple[int, ...] Goal index.
        @return float Combined priority value used in the open set.
        """
        distance=self.heuristics(node,goal)
        safety=self.safetyHeuristics(node,parent)   
        return pathCost*(self.k1+safety) + (self.k2+safety)*distance
        
            
            
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
        @brief Run SDF-guided A* from start to goal on the provided grid.
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
        @note Computes an SDF via distance transform and its gradient, then runs
              an A* loop with a combined distance-and-safety priority.
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
        
        self.distanceTransform=distance_transform_edt(1-self.grid)
        self.grad=np.gradient(self.distanceTransform)
        
        heap=[]
        visited=np.zeros(self.grid.shape,bool)
        parents={}
        g_score = np.full(self.grid.shape, np.inf, np.float32)
        g_score[self.start]=0
        f0=self.combinedHeuristics(self.start,None,0,self.goal)
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
                            total_cost=self.combinedHeuristics(adjacentNodes[k],self.parent,g_updated,self.goal)+g_updated
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
