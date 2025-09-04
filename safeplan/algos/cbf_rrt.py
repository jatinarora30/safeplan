"""
@file rrt.py
@brief Rapidly-exploring Random Tree (RRT) planner for N-dimensional occupancy grids.

@details
Implements a basic RRT over an N-D NumPy occupancy grid where 0 denotes free
space and 1 denotes obstacle. The planner incrementally grows a tree from the
start toward random samples (with goal bias), adding a new node at most
@p stepSize away from the nearest existing node if the local edge is collision-free.
A path is returned once the tree connects to the goal via a collision-free edge.

Collision checking (@ref isEdgeFree()) samples @p pointSamples points along a
candidate segment, rounds them to the nearest grid indices, and rejects the edge
if any sampled index is out of bounds or lies on an obstacle (grid==1).

@par Constructor Arguments
- @p maxIter         : int — maximum number of RRT expansion iterations
- @p goalSampleRate  : float in [0,1] — probability of directly sampling the goal
- @p stepSize        : float — maximum extension (in grid index units) per step
- @p pointSamples    : int — number of samples along an edge for collision checking

@par Inputs (to @ref plan())
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}

@par Outputs
- @p success : int — 1 if a path is found, else 0
- @p path    : list[tuple[int, ...]] — sequence of cells from start→goal (inclusive)
- @p info    : list[str] — diagnostics (e.g., "Invalid start", "Goal has obstacle",
                "Ran out of max iterations couldn't find path")

@note
- Coordinates are treated in grid index space. Steered points are rounded to
  integer indices before validation and insertion.
- The effective step used during steering is clamped by
  min(stepSize, ||goal - start||) to avoid overshoot on very small problems.
- Edge feasibility depends on @p pointSamples; too few samples may miss thin
  obstacles; too many may slow planning.
- Nearest neighbor is a linear scan; for large trees, consider kd-tree/ball-tree.
- RRT is probabilistically complete but not optimal; use RRT* for asymptotic optimality.

@complexity
Per iteration: O(|V|) for nearest neighbor (linear scan) + O(pointSamples) for
edge checking. Worst case ≈ O(maxIter·(|V| + pointSamples)).

@see BasePlanner, AStar
"""

from .baseplanner import BasePlanner
import random
import numpy as np

class CBFRRT(BasePlanner):
    """
    @class RRT
    @brief RRT planner operating on N-D occupancy grids.

    @details
    Maintains a tree of explored, collision-free grid indices. Each iteration:
    1) Sample a random node with goal bias (@ref getRandomNode()).
    2) Find nearest existing node in the tree (@ref getNearestNode()).
    3) Steer from nearest toward random sample by at most @p stepSize.
    4) If the edge is collision-free (@ref isEdgeFree()), add the new node.
    5) If the new node is within @p stepSize of @p goal and the final edge is free,
       terminate with success and reconstruct the path.

    @par Members
    - @p maxIter : int — maximum iterations
    - @p goalSampleRate : float — probability to sample the exact goal
    - @p stepSize : float — max extension length per expansion
    - @p pointSamples : int — samples per edge for collision tests
    - @p success : int — 1 on success, else 0
    - @p info : list[str] — diagnostic messages
    - @p path : list[tuple[int, ...]] — planned path when found
    - @p dimension : int — grid dimensionality (set in @ref plan())
    - @p sizes : list[int] — grid extents per dimension (set in @ref plan())
    - @p start, @p goal, @p grid — problem definition set in @ref plan()
    """

    def __init__(self,maxIter,goalSampleRate,stepSize,pointSamples):
        """
        @brief Construct the class for A* planner

        @post Instance is initialized.
        """
        self.maxIter=maxIter
        self.goalSampleRate = goalSampleRate
        self.stepSize=stepSize
        self.pointSamples=pointSamples
        
        self.success=0
        self.info=[]
        
        self.path= []
        
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
                        if self.grid[tuple(point2)] == 1:
                            return False
        return True

        
    def getRandomNode(self):
        """
        @brief Sample a random node in the grid with goal bias.

        @details
        With probability @p goalSampleRate, returns @p self.goal directly
        (goal bias). Otherwise, uniformly samples an integer index within the
        grid bounds along each dimension.

        @return tuple[int, ...]
                A grid index either equal to @p goal (biased) or a random free index.
        """
        if random.random()<self.goalSampleRate:
            return self.goal
        else:
            point=[]
            for i in range(self.dimension):
                point.append(random.randint(0,(self.sizes[i]-1)))
            return tuple(point)
        
    def dist(self, a, b):
        """
        @brief Euclidean distance between two grid points (in continuous space).

        @param a tuple[int, ...] — first point
        @param b tuple[int, ...] — second point
        @return float — Euclidean norm ||a - b||_2
        """
        return np.linalg.norm(np.array(a) - np.array(b))

        
    def getNearestNode(self,tree, randNode):
        """
        @brief Find the nearest node in the current tree to a query sample.

        @details
        Uses a linear scan over @p tree to compute Euclidean distances
        (@ref dist()) and returns the node with minimum distance.

        @param tree list[tuple[int, ...]] — nodes currently in the RRT
        @param randNode tuple[int, ...] — query/sample node

        @return tuple[int, ...] — nearest node from @p tree to @p randNode

        @complexity O(|tree|) distance computations.
        """
        distances=[]
        for node in tree:
            distances.append(self.dist(node,randNode))
        nearest_node = tree[np.argmin(distances)]
        
        
        return nearest_node
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
        
        parents={}
        tree=[self.start]
        parents[self.start]=None
        for iter in range(self.maxIter):
            randNode=self.getRandomNode()
            nearestNode=self.getNearestNode(tree,randNode)
            steer=np.array(randNode)-np.array(nearestNode)
            steerDir=steer/(np.linalg.norm(steer)+1e-9)
            step = min(self.stepSize, np.linalg.norm(np.array(self.goal) - np.array(self.start)))
            steeredPoint = list(nearestNode + steerDir * step)
            steeredPoint = tuple(int(round(x)) for x in steeredPoint)
            if steeredPoint == nearestNode:
                continue
            
            if not (self.isValid(steeredPoint) and self.isEdgeFree(steeredPoint,nearestNode) and self.grid[steeredPoint]==0):
                continue
            tree.append(steeredPoint)
            parents[steeredPoint]=nearestNode
            
            if self.dist(steeredPoint,self.goal)<self.stepSize:
                
                if self.isEdgeFree(steeredPoint,self.goal):
                    self.success = 1
                    parents[self.goal]=steeredPoint
                    tree.append(self.goal)
                    
            if self.success == 1:
                self.path = []
                current = self.goal
                while current is not None:
                    self.path.append(current)
                    current = parents[current]
                self.path.reverse()
                return self.success, self.path, self.info
                
        
        
        if self.success !=1:
            self.info.append("Ran out of max iterations couldn't find path")
            self.success=0
            self.path=[]
                
            
        
            
        
        return self.success,self.path,self.info
