"""
@file jerk_per_meter.py
@brief Deviation Jerk Per Meter evaluator for planning algorithms

@details
Implements Jerk Per Meter evaluation based upon the path provided on the grid. It uses the third derivate of position with step size of 1

@par Inputs
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}
- @p cellSize  : Size of cell(in m) for real world computation
- @p Path  : Path given by planner to evaluate metrices

@par Outputs
- @p val: Jerk per meter evaluated

@see BaseEval

"""
import numpy as np
from .baseeval import BaseEval
from ..evals.path_cost import PathCost
class JerkPerMeter(BaseEval):
    def __init__(self):
        """
        @brief Construct the class for Jerk per meter evaluator

        @post Instance is initialized.
        """
        self.value=0
        self.pathCost=PathCost()
        
    def eval(self,start,goal,grid,cellSize,path):
        """
        A eval function  for jerk per meter  evaluation, which evaluates on given start, goal, grid, cellSize, and Path returns path cost value
        @param start Takes the n-dimensional start input
        @param goal Takes the n-dimension goal input
        @param grid Takes the N x N dimensional grid
        @param cellSize Takes input as cell size for computation
        @param Path Takes the path from star to goal in the form of a tuple
        @return value Returns the jerk per meter
        
        """
        self.value=0
        data=[]
        if len(path)>3:
            for k in range(len(path)-3):
                data.append((np.array(path[k+3])-3*np.array(path[k+2])+3*np.array(path[k+1])-np.array(path[k]))*cellSize)
            
            pathLen=self.pathCost.eval(None,None,None,cellSize,path)
            data = np.linalg.norm(data, axis=1)
                
            self.value=abs(np.sum(data))/pathLen
                
        else:
            self.value=0
        
        return self.value