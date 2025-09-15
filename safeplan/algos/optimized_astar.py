"""
@file AStar.py
@brief A* planner for N-dimensional occupancy grids.

@details
Implements A* (A-star) search over an N-D NumPy occupancy grid where 0 denotes
free space and 1 denotes an obstacle. The planner takes a start and goal index
(tuples of ints) and returns a collision-free path if one exists. The heuristic
and neighborhood definition are provided by @ref heuristics() and
@ref adjacentCoordinates() and must be chosen consistently.

@par Inputs
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}

@par Outputs
- @p success : int — 1 if a path is found, else 0
- @p path    : list[tuple[int, ...]] — sequence of cells from start→goal (inclusive)
- @p info    : list[str] — diagnostics (e.g., "Invalid goal")

@note Ensure the heuristic matches the move set:
      - 4-neighbors → Manhattan distance
      - 8-neighbors / diagonal moves → Chebyshev (or Euclidean with proper step costs)

@see BasePlanner

@references
- A* overview: https://www.geeksforgeeks.org/python/a-search-algorithm-in-python/
- N-D adjacent coordinates: https://www.geeksforgeeks.org/python/python-adjacent-coordinates-in-n-dimension/


"""


from .baseplanner import BasePlanner
import heapq
import itertools
import numpy as np
from scipy.ndimage import distance_transform_edt

class OptimizedAStar(BasePlanner):
    
    def __init__(self,turnPenaltyCoefficients,safetyDistGridRadius,maxInflateIter,pointSamples):
        """
        @brief Construct the class for A* planner

        @post Instance is initialized.
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
        A function to return if cell is valid or not under the max limits of size of grids and 0 
        @param start Takes the grid cell as input
        @return bool Returns true or false depends on if cell is valid or not
        """
        for i in range(self.dimension):
            if not (0 <= grid_cell[i] < self.grid.shape[i]):
                return False
        return True
    
    def turnHeuristics(self,node1,node2,node3):
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
        A function to return the cost of heuristics between two nodes depending on the eucledian cost 
        @param node1 Takes input first node two get distance between distances
        @param node2 Takes input second node two get distance between distances
        @return cost Returns the eucledian cost between two nodes
        """
        cost=0
        for i in range(0,self.dimension):
            cost+=abs(node1[i]-node2[i])
        return cost
            
            
    def adjacentCoordinates(self,node):
        """
        A function to return the cost of heuristics between two nodes depending on the eucledian cost ,
        itertools.product function is used to compute the directions of adjacent coordinates.
        @param node Takes a node in the input 
        @return cost Returns the adjacent nodes in terms of coordinates
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
        """Raster coefficient P over the hyper-rectangle between start and goal.
           Returns lnP = ln(P) clamped away from 0/1 for numerical stability."""
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
        @brief Check if the segment between two points is collision-free.

        @details
        Samples `self.pointSamples` points linearly between @p pt1 and @p pt2.
        Each sample is rounded to the nearest grid index; if any sample is
        out-of-bounds or falls on an obstacle cell (grid==1), the edge is
        considered in collision.

        @param pt1 sequence[float]
               Segment start (continuous coordinates).
        @param pt2 sequence[float]
               Segment end (continuous coordinates).

        @return bool
                True if all sampled points lie within the grid and in free
                cells; False otherwise.
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
        forward=self.directionalOptimize(self.path)
        backward=self.directionalOptimize(self.path[::-1])[::-1]
        if len(forward)<len(backward):
            return forward
        else:
            return backward

                
        
    def plan(self,start,goal,grid):
        """
        A Plan function  for A*, which plans on given start,goal and grid returns Path, Sucess, info
        @param start Takes the n-dimensional start input
        @param goal Takes the n-dimension goal input
        @param grid Takes the N x N dimensional grid
        @return success Tells if the path was found( as 1 ) or not ( as 0 )
        @return Path Returns the path from star to goal in the form of a tuple
        @return info Returns list of statements of what may may went wrong in finding path from start to goal
        @throws NotImplementedError If a subclass does not override this method.
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
            
                       
        


