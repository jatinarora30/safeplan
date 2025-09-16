"""
@file cbf_rrt.py
@brief Rapidly-exploring Random Tree with Control Barrier Functions (CBF-RRT) for N-D occupancy grids.

@details
Implements a CBF-augmented Rapidly-exploring Random Tree (RRT) over an N-D NumPy
occupancy grid where value 0 denotes free space and value 1 denotes an obstacle.
The planner grows a tree from the start toward random samples (with an adjustable
goal bias). At each step, it proposes a new node no farther than the configured
step size from the nearest existing node. The new node is accepted only if the
local edge is collision-free and a CBF-based safety check passes. A path is
returned once the tree connects to the goal with a collision-free edge.

Collision checking (see @ref isEdgeFree()) samples a fixed number of points along
a candidate segment, rounds them to the nearest grid indices, and rejects the
edge if any sampled index is out of bounds or lies on an obstacle.

The method @ref getObstacleCentersRadii() computes approximate obstacle centers
and radii from connected components. These are used by a grid-based CBF safety
test in @ref isSafe().

@par Constructor Arguments
- @p maxIter         : int — maximum number of RRT expansion iterations
- @p goalSampleRate  : float in [0,1] — probability of directly sampling the goal
- @p stepSize        : float — maximum extension per step in grid index units
- @p pointSamples    : int — number of samples along an edge for collision checking
- @p gamma1          : float — CBF parameter (first-order coefficient)
- @p gamma2          : float — CBF parameter (second-order coefficient)

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
- Coordinates are in grid index space. Steered points are rounded to integer
  indices before validation and insertion.
- The step used during steering is limited by the configured step size and by
  the straight-line distance between start and goal to avoid overshoot on very
  small problems.
- Edge feasibility depends on @p pointSamples; too few may miss thin obstacles,
  too many may slow planning.
- Nearest neighbor is implemented as a linear scan; for large trees, consider a
  spatial index such as a kd-tree or ball-tree.
- RRT is probabilistically complete but not optimal. Use RRT* variants for
  asymptotic optimality (not implemented here).
- The CBF safety check is an approximation derived from obstacle components.

@complexity
Per iteration: linear in the number of existing tree nodes for nearest neighbor,
plus linear in the number of edge samples for collision checking.
Worst case is proportional to the maximum number of iterations multiplied by the
sum of those two costs.

@see BasePlanner, AStar

@references
- @cite CBF_RRT_Hsu2020  Hsu et al., "Safe Path Planning Using Control Barrier Functions,"
  arXiv:2011.06748. https://arxiv.org/pdf/2011.06748
"""

from .baseplanner import BasePlanner
import random
import numpy as np
from scipy.ndimage import label

class CBFRRT(BasePlanner):
    """
    @class CBFRRT
    @brief RRT planner with Control Barrier Function (CBF) safety checks.

    @details
    The planner maintains a tree of collision-free grid indices. Each iteration:
    1) Sample a random node with goal bias (see @ref getRandomNode()).
    2) Find the nearest existing node in the tree (see @ref getNearestNode()).
    3) Steer from the nearest node toward the random sample by at most the step size.
    4) Accept the new node only if both the edge is collision-free (see @ref isEdgeFree())
       and the CBF safety condition passes (see @ref isSafe()).
    5) If the new node is close enough to the goal and the final edge is free,
       declare success and reconstruct the path.

    @par Members
    - @p maxIter : int — maximum iterations
    - @p goalSampleRate : float — probability to sample the exact goal
    - @p stepSize : float — maximum extension length per expansion
    - @p pointSamples : int — samples per edge for collision tests
    - @p gamma1, @p gamma2 : float — CBF coefficients
    - @p success : int — 1 on success, else 0
    - @p info : list[str] — diagnostic messages
    - @p path : list[tuple[int, ...]] — planned path when found
    - @p dimension : int — grid dimensionality (set in @ref plan())
    - @p sizes : list[int] — grid extents per dimension (set in @ref plan())
    - @p start, @p goal, @p grid — problem definition set in @ref plan()
    """

    def __init__(self,maxIter,goalSampleRate,stepSize,pointSamples,gamma1,gamma2):
        """
        @brief Construct the CBF-RRT planner.
        @param maxIter int Maximum number of RRT iterations.
        @param goalSampleRate float Goal sampling probability in [0,1].
        @param stepSize float Maximum extension per step in grid index units.
        @param pointSamples int Number of samples per edge for collision checking.
        @param gamma1 float CBF parameter (first-order coefficient).
        @param gamma2 float CBF parameter (second-order coefficient).
        @post Instance is initialized; outputs are cleared.
        """
        self.maxIter=maxIter
        self.goalSampleRate = goalSampleRate
        self.stepSize=stepSize
        self.pointSamples=pointSamples
        self.gamma1=gamma1
        self.gamma2=gamma2
        
        self.success=0
        self.info=[]
        
        self.path= []
        
    def isEdgeFree(self, pt1, pt2):
        """
        @brief Check if the segment between two points is collision-free.
        @details
        Samples a fixed number of points linearly between the endpoints.
        Each sample is rounded to the nearest grid index. The edge is rejected
        if any sampled index is out of bounds or lies on an obstacle cell.
        @param pt1 sequence[float] Segment start in continuous coordinates.
        @param pt2 sequence[float] Segment end in continuous coordinates.
        @return bool True if all sampled points are valid and free; otherwise False.
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
        With probability equal to the goal sampling rate, returns the goal directly.
        Otherwise, samples a random integer index within the grid bounds along each
        dimension.
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
        @return float Euclidean distance between the two points.
        """
        return np.linalg.norm(np.array(a) - np.array(b))

        
    def getNearestNode(self,tree, randNode):
        """
        @brief Find the nearest node in the current tree to a query sample.
        @details
        Performs a linear scan over the tree to compute distances and returns the
        node with the minimum distance to the query sample.
        @param tree list[tuple[int, ...]] Nodes currently in the RRT.
        @param randNode tuple[int, ...] Query or sample node.
        @return tuple[int, ...] Nearest node from the tree to the query node.
        """
        distances=[]
        for node in tree:
            distances.append(self.dist(node,randNode))
        nearest_node = tree[np.argmin(distances)]
        
        
        return nearest_node
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
    
    def getObstacleCentersRadii(self):
        """
        @brief Find centers and radii of connected obstacle components.
        @details
        Labels connected components of obstacle cells. For each component:
        the center is the mean of its cell indices (centroid), and the radius
        is the largest Euclidean distance from this centroid to any cell in
        the component. These values provide a coarse approximation of obstacle
        geometry for use in the CBF check.
        @return (numpy.ndarray, numpy.ndarray)
                An array of centers with shape (num_components, dimension) and
                an array of radii with shape (num_components,).
        """
        labeled_grid, num_features = label(self.grid)
        centers, radii = [], []
        for label_num in range(1, num_features + 1):
            positions = np.argwhere(labeled_grid == label_num)  # cells in this obstacle
            center = positions.mean(axis=0)                     # centroid
            # compute radius as farthest cell from centroid
            if len(positions) > 0:
                dists = np.linalg.norm(positions - center, axis=1)
                radius = dists.max()
            else:
                radius = 0.0
            centers.append(center)
            radii.append(radius)

        return np.array(centers), np.array(radii)
    
    def heuristics(self,node1,node2):
        """
        @brief Squared Euclidean proxy between two nodes.
        @param node1 tuple[int, ...] First node.
        @param node2 tuple[int, ...] Second node.
        @return float Sum of squared differences across all coordinates.
        @note Used internally by the safety check.
        """
        cost=0
        for i in range(0,self.dimension):
            cost+=np.square(abs(node1[i]-node2[i]))
        return cost
    
    def isSafe(self,cell,vel):
        """
        @brief Control Barrier Function (CBF) safety check for a candidate cell and velocity.
        @details
        Uses the obstacle component centers and radii computed by @ref getObstacleCentersRadii().
        For each obstacle, a barrier value is formed from the squared distance to the
        obstacle center minus the squared radius. A change rate is computed using the
        steer vector. Two user-supplied parameters (gamma1 and gamma2) combine these
        quantities into a safety measure. If the measure indicates increasing safety
        or adequate safety margin for any obstacle component, the candidate state is
        considered safe for insertion into the tree.
        @param cell tuple[int, ...] Candidate grid cell.
        @param vel sequence[float] Steer vector used to reach the candidate cell.
        @return bool True if the CBF condition passes; otherwise False.
        @note This is an approximate, grid-based CBF that uses component centroids and radii.
        """
        for i in range(len(self.radius)):
            b=self.heuristics(cell,self.obsctaclesCenters[i])-np.square(self.radius[i])
            bDot=2*np.dot(np.array(cell)-np.array(self.obsctaclesCenters[i]),vel)
            b1=b*self.gamma1+bDot
            b1Dot=self.gamma1*bDot+2*np.sum(np.square(vel))    
            if b1Dot+self.gamma2*b1>=0:
                return True
        return False
    
    def plan(self,start,goal,grid):
        """
        @brief Plan with CBF-RRT on the provided grid from start to goal.
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
        self.obsctaclesCenters,self.radius=self.getObstacleCentersRadii()
        
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
            if not self.isSafe(steeredPoint,steer):
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
