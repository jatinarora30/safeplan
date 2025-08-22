"""
@file distance_to_goal.py
@brief It tells how much distance to goal if path is incomplete or no path  for planning algorithms

@details
Implements distance bwtween last node of path and goal and if no path between start and goal

@par Inputs
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}
- @p cellSize  : Size of cell(in m) for real world computation
- @p Path  : Path given by planner to evaluate metrices

@par Outputs
- @p val: Distance to goal in terms of failure

@see BaseEval

"""
from .baseeval import BaseEval
import math
class DistanceToGoal(BaseEval):
    def __init__(self):
        """
        @brief Construct the class for DistanceToGoal evaluator

        @post Instance is initialized.
        """
        self.value=0
        
    def eval(self,start,goal,grid,cellSize,path):
        """
        A eval function  for distane to goal evaluation, which evaluates on given start, goal, grid, cellSize, and Path returns path cost value
        @param start Takes the n-dimensional start input
        @param goal Takes the n-dimension goal input
        @param grid Takes the N x N dimensional grid
        @param cellSize Takes input as cell size for computation
        @param Path Takes the path from star to goal in the form of a tuple
        @return value Returns the distance to goal in failure case
        
        """
        if len(path)>0:
            self.value = math.dist(path[len(path)-1],goal)
        else:
            self.value = math.dist(start,goal)
        
        return self.value