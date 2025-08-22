"""
@file optimal_deviation.py
@brief Deviation path length percent evaluator for planning algorithms

@details
Implements Optimal deviation evaluation based upon the path provided on the grid.

@par Inputs
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}
- @p cellSize  : Size of cell(in m) for real world computation
- @p Path  : Path given by planner to evaluate metrices

@par Outputs
- @p val: Path deviated from optimal path

@see BaseEval

"""
from .baseeval import BaseEval
from ..algos.a_star import AStar
from ..evals.path_cost import PathCost
class OptimalDeviation(BaseEval):
    def __init__(self):
        """
        @brief Construct the class for Optimal Deviation evaluator

        @post Instance is initialized.
        """
        self.value=0
        self.planner=AStar()
        self.pathCost=PathCost()
        
    def eval(self,start,goal,grid,cellSize,path):
        """
        A eval function  for Optimal deviation evaluation, which evaluates on given start, goal, grid, cellSize, and Path returns path cost value
        @param start Takes the n-dimensional start input
        @param goal Takes the n-dimension goal input
        @param grid Takes the N x N dimensional grid
        @param cellSize Takes input as cell size for computation
        @param Path Takes the path from star to goal in the form of a tuple
        @return value Returns the optimal deviation in percent
        
        """
        optimalPath=self.planner.plan(start,goal,grid)
        optimalCost=self.pathCost.eval(None,None,None,cellSize,optimalPath[1])
        originalCost=self.pathCost.eval(None,None,None,cellSize,path)
        if optimalCost !=0:
            self.value=((originalCost-optimalCost)/(optimalCost))
        else:
            self.value=0
        
        return self.value