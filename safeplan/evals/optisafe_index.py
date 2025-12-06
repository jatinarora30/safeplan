"""
@file optisafe_index.py
@brief Optisafe Index evaluator for planning algorithms

@details
Implements OptiSafe Index evaluation based upon the path provided on the grid.

@par Inputs
- @p start : tuple[int, ...] — start grid cell (e.g., (row, col))
- @p goal  : tuple[int, ...] — goal grid cell
- @p grid  : numpy.ndarray (N-D), values {0=free, 1=obstacle}
- @p cellSize  : Size of cell(in m) for real world computation
- @p Path  : Path given by planner to evaluate metrices

@par Outputs
- @p val: OptiSafe Index

@see BaseEval

"""
from .baseeval import BaseEval
from ..algos.a_star import AStar
from ..algos.voronoi_planner import VoronoiPlanner
from ..algos.upp import UPP
from ..evals.path_cost import PathCost
from ..evals.minimum_clearance import MinimumClearance
import numpy as np
class OptiSafeIndex(BaseEval):
    def __init__(self,pointSamples,knn):
        """
        @brief Construct the class for OptiSafe Index evaluator

        @post Instance is initialized.
        """
        self.value=0
        import numpy as np

        self.upp = UPP(
            alphaBase=0.5,
            betaBase=10,
            radiusBase=30,
            epsilon=0.01,

            # β adaptivity parameters
            betaMin=0.1,
            betaMax=2.0,
            betaDecay=0.97,
            betaRecovery=1.05,
            betaPatience=20,
            goalTol=0.1,

            # α adaptivity parameters
            alphaMin=0.05,
            alphaMax=0.95,
            alphaDecay=0.97,
            alphaRecovery=1.05,
            tolAngular=np.deg2rad(180),
            turnTarget=np.deg2rad(15),
            turnWindow=10,

            # radius adaptivity parameters
            radiusMin=1,
            radiusMax=5
        )

        self.aStar=AStar()
        self.vornoiPlanner=VoronoiPlanner(pointSamples=pointSamples,knn=knn)
        self.pathCost=PathCost()
        self.minimumClearance=MinimumClearance(pointSamples=pointSamples)
        
    def eval(self,start,goal,grid,cellSize,path):
        """
        A eval function  for OptiSafe Index evaluation, which evaluates on given start, goal, grid, cellSize, and Path returns path cost value
        @param start Takes the n-dimensional start input
        @param goal Takes the n-dimension goal input
        @param grid Takes the N x N dimensional grid
        @param cellSize Takes input as cell size for computation
        @param Path Takes the path from star to goal in the form of a tuple
        @return value Returns the OptiSafe Index 
        
        """
        self.value=0
        optimalPath=self.aStar.plan(start,goal,grid)
        optimalCost=self.pathCost.eval(None,None,None,cellSize,optimalPath[1])
        originalCost=self.pathCost.eval(None,None,None,cellSize,path)
        safeVornoiPath=self.vornoiPlanner.plan(start,goal,grid)
        if safeVornoiPath[0]==0:
            safePath=self.upp.plan(start,goal,grid)
        else:
            safePath=safeVornoiPath
            

        if safePath[0]==1:
            minimumSafe=self.minimumClearance.eval(start,goal,grid,cellSize,safePath[1])
            originalSafe=self.minimumClearance.eval(start,goal,grid,cellSize,path)
        
        
            if minimumSafe>0:
                if(originalSafe>minimumSafe):
                    sd=0
                else:
                    sd=(minimumSafe-originalSafe)/minimumSafe
                    
            else:
                sd=1
        else:
            sd=1
            
        if optimalCost>0:
            
                od=min(1,max(((originalCost-optimalCost)/optimalCost),0))
                
        else:
            od=1
        
        o=1-od
        s=1-sd
        self.value=((1-abs(o-s))*np.sqrt(o*o+s*s))/np.sqrt(2)
        
        
        return self.value