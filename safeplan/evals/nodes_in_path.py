"""
@file nodes_in_path.py
@brief Nodes in path evaluator for planning algorithms

@details
Implements Nodes in path evaluation based upon the path provided on the grid. 


@par Inputs
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}
- @p cellSize  : Size of cell(in m) for real world computation
- @p Path  : Path given by planner to evaluate metrices

@par Outputs
- @p val: Nodes in Path

@see BaseEval

"""
import numpy as np
from .baseeval import BaseEval
from rdp import rdp
class NodesInPath(BaseEval):
    def __init__(self,type,epsilon):
        """
        @brief Construct the class for Nodes in Path evaluator
        @param type Takes the input if it is calculating simple length, or epslon smoothening for length "Simple" or "RDP"
        @param epsilon tolerence for RDP if not simple
        @post Instance is initialized.
        """
        self.type=type
        self.value=0
        self.epsilon=epsilon
        
    def eval(self,start,goal,grid,cellSize,path):
        """
        A eval function  for number of nodes evaluation, which evaluates on given start, goal, grid, cellSize, and Path returns path cost value
        @param start Takes the n-dimensional start input
        @param goal Takes the n-dimension goal input
        @param grid Takes the N x N dimensional grid
        @param cellSize Takes input as cell size for computation
        @param Path Takes the path from star to goal in the form of a tuple
        @return value Returns the number of nodes in path based on type
        
        """
        self.value=0
        if self.type=="Simple":
            self.value=len(path)
            
        if self.type=="RDP":
            
            self.value=len(rdp(path,self.epsilon))
            
        
        return self.value
