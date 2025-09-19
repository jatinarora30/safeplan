"""
@file rrt.py
@brief Rapidly-exploring Random Tree (RRT) planner for N-dimensional occupancy grids.

@details
Implements a basic RRT over an N-D NumPy occupancy grid where value 0 denotes
free space and value 1 denotes an obstacle. The planner incrementally grows a
tree from the start toward random samples (with goal bias). At each step, it
adds a new node no farther than the configured step size from the nearest
existing node, provided the local edge is collision-free. A path is returned
once the tree connects to the goal via a collision-free edge.

Collision checking (see @ref isEdgeFree()) samples a fixed number of points
along a candidate segment, rounds them to the nearest grid indices, and rejects
the edge if any sampled index is out of bounds or lies on an obstacle cell.

@par Constructor Arguments
- @p maxIter         : int — maximum number of RRT expansion iterations
- @p goalSampleRate  : float in [0,1] — probability of directly sampling the goal
- @p stepSize        : float — maximum extension per step in grid index units
- @p pointSamples    : int — number of samples along an edge for collision checking

@par Inputs (to @ref plan())
- @p start : tuple[int, ...] — start grid cell (for example, (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}

@par Outputs
- @p success : int — 1 if a path is found, else 0
- @p path    : list[tuple[int, ...]] — sequence of cells from start to goal (inclusive)
- @p info    : list[str] — diagnostics (for example, "Invalid start", "Goal has obstacle",
                "Ran out of max iterations couldn't find path")

@note
- Coordinates are treated in grid index space. Steered points are rounded to
  integer indices before validation and insertion.
- The effective step during steering is clamped by the minimum of the step size
  and the straight-line distance between start and goal to avoid overshoot on
  very small problems.
- Edge feasibility depends on @p pointSamples; too few samples may miss thin
  obstacles, while too many may slow planning.
- Nearest neighbor is a linear scan; for large trees, consider a spatial index
  such as a kd-tree or ball-tree.
- RRT is probabilistically complete but not optimal; use RRT* for asymptotic
  optimality (not implemented here).

@complexity
Per iteration: linear in the number of current tree nodes for nearest neighbor,
plus linear in the number of edge samples for collision checking. Worst case is
proportional to the maximum number of iterations multiplied by the sum of those
two costs.

@see BasePlanner, AStar

@references
- LaValle, S., “Rapidly-exploring Random Trees: A New Tool for Path Planning.”
  Overview link: https://www.semanticscholar.org/paper/Rapidly-exploring-random-trees-%3A-a-new-tool-for-LaValle/d967d9550f831a8b3f5cb00f8835a4c866da60ad
"""

from .baseplanner import BasePlanner
import random
import numpy as np

class VRRT(BasePlanner):
    """
    @class RRT
    @brief RRT planner operating on N-D occupancy grids.

    @details
    The planner maintains a tree of explored, collision-free grid indices. Each
    iteration:
    1) Sample a random node with goal bias (see @ref getRandomNode()).
    2) Find the nearest existing node in the tree (see @ref getNearestNode()).
    3) Steer from the nearest node toward the random sample by at most the step size.
    4) If the edge is collision-free (see @ref isEdgeFree()), add the new node.
    5) If the new node is close enough to the goal and the final edge is free,
       declare success and reconstruct the path.

    @par Members
    - @p maxIter : int — maximum iterations
    - @p goalSampleRate : float — probability to sample the exact goal
    - @p stepSize : float — maximum extension length per expansion
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
        @brief Construct the RRT planner.
        @param maxIter int Maximum number of RRT iterations.
        @param goalSampleRate float Goal sampling probability in [0,1].
        @param stepSize float Maximum extension per step in grid index units.
        @param pointSamples int Number of samples per edge for collision checking.
        @post Instance is initialized; outputs are cleared.
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
        Samples a fixed number of points linearly between the endpoints. Each
        sample is rounded to the nearest grid index. The edge is rejected if
        any sampled index is out of bounds or lies on an obstacle cell.
        @param pt1 sequence[float] Segment start in continuous coordinates.
        @param pt2 sequence[float] Segment end in continuous coordinates.
        @return bool True if all sampled points lie within bounds and in free cells.
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
        With probability equal to the goal sampling rate, returns the goal
        directly. Otherwise, uniformly samples an integer index within the grid
        bounds along each dimension.
        @return tuple[int, ...] A grid index equal to the goal (biased) or a random index.
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
        @brief Euclidean distance between two grid points in continuous space.
        @param a tuple[int, ...] First point.
        @param b tuple[int, ...] Second point.
        @return float Straight-line distance between the two points.
        """
        return np.linalg.norm(np.array(a) - np.array(b))

        
    def getNearestNode(self,tree, randNode):
        """
        @brief Find the nearest node in the current tree to a query sample.
        @details
        Uses a linear scan over the tree, computes Euclidean distances with
        @ref dist(), and returns the node with the smallest distance.
        @param tree list[tuple[int, ...]] Nodes currently in the RRT.
        @param randNode tuple[int, ...] Query or sample node.
        @return tuple[int, ...] Nearest node from the tree to the query node.
        @complexity Linear in the number of nodes in the tree.
        """
        distances=[]
        for node in tree:
            distances.append(self.dist(node,randNode))
        nearest_node = tree[np.argmin(distances)]
        
        
        return nearest_node
    
    def lqrCBFSteer(self,nodeStart,nodeNext):
        xNew=None


        return xNew

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
    def plan(self,start,goal,grid):
        """
        @brief Plan with RRT on the provided grid from start to goal.
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
        - "Ran out of max iterations couldn't find path"
        @note Uses linear-scan nearest neighbor and simple steering with a step clamp.
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
