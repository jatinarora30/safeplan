"""
@file path_cost.py
@brief Path cost evaluator for planning algorithms

@details
Implements Path cost evaluation based upon the path provided on the grid. 
It calculates eucledian distance between nodes for N - dimensional path

@par Inputs
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}
- @p cellSize  : Size of cell(in m) for real world computation
- @p Path  : Path given by planner to evaluate metrices

@par Outputs
- @p val: Path cost computed

@see BaseEval

"""
import numpy as np
from .baseeval import BaseEval
import math
class PathCost(BaseEval):
    def __init__(self):
        """
        @brief Construct the class for Path cost evaluator

        @post Instance is initialized.
        """
        self.value=0
        
    def eval(self,start,goal,grid,cellSize,path):
        """
        A eval function  for Path cost evaluation, which evaluates on given start, goal, grid, cellSize, and Path returns path cost value
        @param start Takes the n-dimensional start input
        @param goal Takes the n-dimension goal input
        @param grid Takes the N x N dimensional grid
        @param cellSize Takes input as cell size for computation
        @param Path Takes the path from star to goal in the form of a tuple
        @return value Returns the path cost
        
        """
        self.value=0
        for k in range(len(path)-1):
            val = math.dist(path[k],path[k+1])
            self.value+=val
        return self.value * cellSize