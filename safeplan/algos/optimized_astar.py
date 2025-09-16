"""
@file optimized_astar.py
@brief Optimized A* planner for N-dimensional occupancy grids.

@details
Implements an A* planner with additional path-quality and safety refinements on
an N-D NumPy occupancy grid where value 0 denotes free space and value 1 denotes
an obstacle. The planner accepts start and goal grid indices and returns a
collision-free path if one exists.

Key features beyond a standard A*:
- Turn-aware cost: adds a penalty based on local change in heading to discourage
  sharp turns (see @ref turnHeuristics()).
- Safety inflation: uses a distance transform to nudge path points away from
  nearby obstacles (see @ref safeInflate()).
- Path shortcutting: performs forward and backward line-of-sight pruning to
  reduce unnecessary waypoints (see @ref directionalOptimize() and
  @ref bidirectionalOptimize()).
- Local edge collision checking: samples points along a segment and rejects it if
  any samples fall out of bounds or on obstacles within a safety radius
  (see @ref isEdgeFree()).

Heuristic and neighborhood generation are provided by @ref heuristics() and
@ref adjacentCoordinates(). Ensure the heuristic is compatible with the allowed
moves and step costs.

@par Constructor Arguments
- @p turnPenaltyCoefficients : float — multiplier for penalizing local turning
- @p safetyDistGridRadius    : float — minimum desired clearance from obstacles in grid cells
- @p maxInflateIter          : int — maximum iterations for safety inflation per waypoint
- @p pointSamples            : int — number of samples along an edge for collision checking

@par Inputs (to @ref plan())
- @p start : tuple[int, ...] — start grid cell (for example, (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}

@par Outputs
- @p success : int — 1 if a path is found, else 0
- @p path    : list[tuple[int, ...]] — sequence of cells from start to goal (inclusive)
- @p info    : list[str] — diagnostics (for example, "Invalid goal")

@note
- Coordinates are handled in grid index space. Steered or optimized points are
  rounded to integer indices before validation.
- The distance transform is computed once per plan call and reused in collision
  checks and safety inflation.
- The neighborhood includes all immediate offsets except the zero vector.
- Only axis-aligned moves are expanded in the inner loop of @ref plan() to keep
  turning penalties predictable; adjust as needed for your benchmark.

@see BasePlanner

@references
- Optimized A* with safety-aware planning ideas:
  https://www.mdpi.com/1424-8220/24/10/3149
"""

from .baseplanner import BasePlanner
import heapq
import itertools
import numpy as np
from scipy.ndimage import distance_transform_edt

class OptimizedAStar(BasePlanner):
    
    def __init__(self,turnPenaltyCoefficients,safetyDistGridRadius,maxInflateIter,pointSamples):
        """
        @brief Construct the Optimized A* planner.
        @param turnPenaltyCoefficients float Multiplier for local turn penalty.
        @param safetyDistGridRadius float Minimum clearance target in grid cells.
        @param maxInflateIter int Maximum iterations for safety inflation per waypoint.
        @param pointSamples int Samples per edge for collision checking.
        @post Instance is initialized; outputs are cleared.
        """
        
        self.success=0
        self.turnPenaltyCoefficients=turnPenaltyCoefficients
        self.safetyDistGridRadius=safetyDistGridRadius
        self.maxInflateIter=maxInflateIter
        self.pointSamples=pointSamples
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
    
    def turnHeuristics(self,node1,node2,node3):
        """
        @brief Estimate the local turn angle between two consecutive segments.
        @details
        Computes the angle between the direction from @p node1 to @p node2 and the
        direction from @p node2 to @p node3. Returns the angle in degrees. If any
        of the inputs are missing, returns zero.
        @param node1 tuple[int, ...] Previous waypoint.
        @param node2 tuple[int, ...] Current waypoint.
        @param node3 tuple[int, ...] Next waypoint.
        @return float Estimated heading change in degrees.
        """
        if node1==None or node2==None or node3==None:
            return 0
        
        v1=np.array(node2)-np.array(node1)
        v2=np.array(node3)-np.array(node2)
        v1Norm=v1/(np.linalg.norm(v1)+1e-9)
        v2Norm=v2/(np.linalg.norm(v2)+1e-9)
        dot_prod = max(-1.0, min(1.0, np.dot(v1Norm, v2Norm)))
        angle_rad = np.arccos(dot_prod)  
        return np.degrees(angle_rad)

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
    
    def calculateRasterCoeffiecient(self):
        """
        @brief Compute a raster coefficient over the axis-aligned box between start and goal.
        @details
        Extracts the hyper-rectangle spanning start and goal, counts obstacle cells,
        and returns the natural logarithm of the obstacle ratio. The value is clamped
        away from zero and one for numerical stability.
        @return float Natural log of the clamped obstacle ratio in the rectangle.
        """
        mins = tuple(min(s, g) for s, g in zip(self.start, self.goal))
        maxs = tuple(max(s, g) for s, g in zip(self.start, self.goal))

        # slice the N-D subarray
        slices = tuple(slice(lo, hi + 1) for lo, hi in zip(mins, maxs))
        rect = self.grid[slices]

        N = int(rect.sum())  # 1 = obstacle, 0 = free
        # total cells in the hyper-rectangle
        A = 1
        for lo, hi in zip(mins, maxs):
            A *= (hi - lo + 1)

        P = N / max(1, A)
        # clamp strictly inside (0,1) so ln is defined
        P = float(min(max(P, 1e-6), 1.0 - 1e-6))
        return float(np.log(P))
    
    def safeInflate(self):
        """
        @brief Push intermediate waypoints away from obstacles using a distance transform.
        @details
        Computes a distance transform on the free-space mask and attempts to replace
        intermediate path points with nearby points that have larger clearance,
        up to a configured target radius. The start and goal are preserved.
        @return list[tuple[int, ...]] Updated path after safety inflation.
        """
        self.distanceTransform=distance_transform_edt(1-self.grid)
        
        if len(self.path)<3:
            return self.path
        self.updatedPath=[]
        self.updatedPath.append(self.path[0])
        for node in self.path[1:-1]:
            if self.distanceTransform[node]>self.safetyDistGridRadius+0.5:
                self.updatedPath.append(node)
            else:
                maxDist=None
                maxDist,maxDistCoord=-np.inf,node
                for i in range(self.maxInflateIter):
                    adjCoord=self.adjacentCoordinates(maxDistCoord)
                    for adj in adjCoord:
                        if self.isValid(adj) and self.grid[adj]==0:
                            if self.distanceTransform[adj]>maxDist:
                                maxDist=abs(self.distanceTransform[adj])
                                maxDistCoord=adj
                    if self.distanceTransform[maxDistCoord]>self.safetyDistGridRadius+0.5:
                        break
                self.updatedPath.append(maxDistCoord)
        
        self.updatedPath.append(self.path[-1])
        return self.updatedPath
    
    def isEdgeFree(self, pt1, pt2):
        """
        @brief Check if the segment between two points is collision-free with clearance.
        @details
        Samples a fixed number of points linearly between the endpoints. Each sample
        is rounded to the nearest grid index. The edge is rejected if any sampled
        index is out of bounds, lies on an obstacle, or has distance-to-obstacle
        below the configured safety radius.
        @param pt1 sequence[float] Segment start in continuous coordinates.
        @param pt2 sequence[float] Segment end in continuous coordinates.
        @return bool True if all sampled points are valid, free, and safely clear.
        """
        for t in np.linspace(0, 1, self.pointSamples):
            point = (1 - t) * np.array(pt1) + t * np.array(pt2)

            valid = True
            for p in range(self.dimension):
                if not (0 <= point[p] < self.grid.shape[p]):
                    valid = False
                    break
                if valid:
                    point2 = []
                    for k in range(self.dimension):
                        point2.append(int(round(point[k])))
                    if self.isValid(tuple(point2)):
                        if self.grid[tuple(point2)] == 1 and self.distanceTransform[tuple(point2)]<self.safetyDistGridRadius+0.5:
                            return False
        return True
    
    def directionalOptimize(self,path):
        """
        @brief Shortcut a path by removing unnecessary waypoints in one direction.
        @details
        Starting at the first point, repeatedly select the farthest point that is
        directly reachable with a collision-free edge, then continue from there.
        @param path list[tuple[int, ...]] Path to optimize.
        @return list[tuple[int, ...]] Optimized path with fewer waypoints.
        """
        if len(path) < 3:
            return path

        optimized = [path[0]]
        i = 0
        while i < len(path) - 1:
            j_best = i + 1
            # Try farthest j first, then closer ones
            for j in range(len(path)-1, i, -1):
                if self.isEdgeFree(path[i],path[j]):
                    j_best = j
                    break
            optimized.append(path[j_best])
            i = j_best  # jump to the farthest safe node

        return optimized
    
    
    def bidirectionalOptimize(self):
        """
        @brief Apply directional shortcutting in both directions and choose the better result.
        @details
        Runs @ref directionalOptimize() forward and backward, then returns the
        shorter of the two results.
        @return list[tuple[int, ...]] Path after bidirectional shortcutting.
        """
        forward=self.directionalOptimize(self.path)
        backward=self.directionalOptimize(self.path[::-1])[::-1]
        if len(forward)<len(backward):
            return forward
        else:
            return backward

                
        
    def plan(self,start,goal,grid):
        """
        @brief Run Optimized A* from start to goal on the provided grid.
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
        @note The method computes a raster coefficient for environmental clutter,
              uses an A* loop with turn-aware total cost, inflates the path for
              safety, and finally applies bidirectional shortcutting.
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
        self.lnP=self.calculateRasterCoeffiecient()
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
                        if is_axis_step:
                            g_updated = g + 1
                            if g_updated < g_score[adjacentNodes[k]]:
                                g_score[adjacentNodes[k]] = g_updated
                                
                                turnHeur=self.turnHeuristics(parents.get(self.parent),self.parent,adjacentNodes[k])
                                
                                total_cost=(1-self.lnP)*self.heuristics(adjacentNodes[k],self.goal)+self.turnPenaltyCoefficients*turnHeur+g_updated
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
            self.path=self.safeInflate()
            self.path=self.bidirectionalOptimize()
            
        
        return self.success,self.path,self.info
