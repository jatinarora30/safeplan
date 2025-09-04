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

class Dijkstra(BasePlanner):
    
    def __init__(self):
        """
        @brief Construct the class for A* planner

        @post Instance is initialized.
        """
        
        self.success=0
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
            
                       
        


