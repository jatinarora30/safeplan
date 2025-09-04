"""
@file minimum_clearance.py
@brief Minimum Clearance evaluator for planning algorithms

@details
Implements Minimum Clearance evaluation based upon the path provided on the grid. 
It calculates eucledian distance between nodes for N - dimensional path

@par Inputs
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}
- @p cellSize  : Size of cell(in m) for real world computation
- @p Path  : Path given by planner to evaluate metrices

@par Outputs
- @p val: Minimum Clearance computed

@see BaseEval

"""
from scipy.ndimage import distance_transform_edt
import numpy as np
from .baseeval import BaseEval
class MinimumClearance(BaseEval):
    def __init__(self,pointSamples):
        """
        @brief Construct the class for Minimum Clearance evaluator
        @param pointSamples Takes input number of point samples between 2 points to calculate distance
        @post Instance is initialized.
        """
        self.value=0
        self.pointSamples=pointSamples
   
        
    def eval(self,start,goal,grid,cellSize,path):
        """
        A eval function  for Minimum Clearance evaluation, which evaluates on given start, goal, grid, cellSize, and Path returns Minimum Clearance value
        @param start Takes the n-dimensional start input
        @param goal Takes the n-dimension goal input
        @param grid Takes the N x N dimensional grid
        @param cellSize Takes input as cell size for computation
        @param Path Takes the path from star to goal in the form of a tuple
        @return value Returns the Minimum Clearance
        
        """
        self.value=0
        distanceTransform=distance_transform_edt(1-grid)
        distances=[]
        self.dimension=len(grid.shape)
        
        if len(path)>=2:
        
            for i in range(len(path)-1):
                start,end=np.array(path[i]),np.array(path[i+1])
                for t in np.linspace(0, 1, self.pointSamples):
                    point= t*start+(1-t)*end
                    valid=True
                    for p in range(self.dimension):
                        if not (0 <= point[p] < grid.shape[p]):
                            valid=False
                            break
                    if valid:
                        point2=[]
                        for k in range(self.dimension):
                            point2.append(int(round(point[k])))
                        distances.append(distanceTransform[tuple(point2)])
            self.value=np.min(distances)*cellSize
        
        return self.value