"""
@file turning_angle.py
@brief Turning Angle evaluator for planning algorithms

@details
Implements Turning Angle evaluation based upon the path provided on the grid. It uses the third derivate of position with step size of 1

@par Inputs
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}
- @p cellSize  : Size of cell(in m) for real world computation
- @p Path  : Path given by planner to evaluate metrices

@par Outputs
- @p val: Turning Angle evaluated

@see BaseEval

"""
import numpy as np
from .baseeval import BaseEval
class TurningAngle(BaseEval):
    def __init__(self):
        """
        @brief Construct the class for Turning Angle evaluator

        @post Instance is initialized.
        """
        self.value=0
        
    def eval(self,start,goal,grid,cellSize,path):
        """
        A eval function  for Turning Angle  evaluation, which evaluates on given start, goal, grid, cellSize, and Path returns path cost value
        @param start Takes the n-dimensional start input
        @param goal Takes the n-dimension goal input
        @param grid Takes the N x N dimensional grid
        @param cellSize Takes input as cell size for computation
        @param Path Takes the path from star to goal in the form of a tuple
        @return value Returns the Turning Angle
        
        """
        self.value=0
        if len(path)>3:
            for k in range(1,len(path)-1):
                v1=np.array(path[k])-np.array(path[k-1])
                v2=np.array(path[k+1])-np.array(path[k])
                v1Norm=v1/(np.linalg.norm(v1)+1e-9)
                v2Norm=v2/(np.linalg.norm(v2)+1e-9)
                dot_prod = max(-1.0, min(1.0, np.dot(v1Norm, v2Norm)))
                angle_rad = np.arccos(dot_prod)
                self.value += np.degrees(angle_rad)
                
        else:
            self.value=0
        
        return self.value