import tracemalloc
import json
import os
import time
from .core.logger import Logger

from .algos.a_star import AStar
from .algos.voronoi_planner import VoronoiPlanner
from .algos.upp import UPP
from .algos.rrt import RRT
from .algos.dijkstra import Dijkstra
from .algos.weighted_astar import WeightedAStar
from .algos.sdf_astar import SDFAStar
from .algos.cbf_rrt import CBFRRT
from .algos.optimized_astar import OptimizedAStar

from .envs.generate_grid import GenerateGrid

from .evals.path_cost import PathCost
from .evals.nodes_in_path import NodesInPath
from .evals.optimal_deviation import OptimalDeviation
from .evals.distance_to_goal import DistanceToGoal
from .evals.jerk_per_meter import JerkPerMeter
from .evals.turning_angle import TurningAngle
from .evals.minimum_clearance import MinimumClearance
from .evals.average_minimum_clearance import AverageMinimumClearance
from .evals.clearance_variability import ClearanceVariability
from .evals.danger_violations import DangerViolations
from .evals.optisafe_index import OptiSafeIndex

class SafePlan:
    def __init__(self,runConfigPath):
        self.runConfigPath=runConfigPath
        
        with open(self.runConfigPath) as file:
            data = json.load(file)
        
        self.outputDir="outputs"
        self.envsDetails=data["envDetails"]
        self.algosDetails=data["algoDetails"]
        self.evalsDetails=data["evalDetails"]
        self.runDetails=data["runDetails"]
        self.iteration=0
        self.isSuccessEval=0
        self.isPlanningTimeEval=0
        self.isPeakMemoryEval=0
        
        # classes objects lists
        self.envs=[]
        self.algos=[]
        self.evals=[]
        self.scenerios=[]
        

        
        self.setUpAlgos()
        self.setUpEvals()
        self.setUpEnvs()
        
        self.logger = Logger(self.runDetails,self.algos,self.outputDir)
        self.runPath=os.path.join(self.outputDir,self.runDetails)
        
        
        
    def setUpAlgos(self):
        for k in self.algosDetails:
            if k["name"]=="AStar":
                self.algos.append(("AStar",AStar()))
            if k["name"]=="OptimizedAStar":
                self.algos.append(("OptimizedAStar",OptimizedAStar(k["args"]["turnPenaltyCoefficients"],k["args"]["safetyDistGridRadius"],k["args"]["maxInflateIter"],k["args"]["pointSamples"])))
            if k["name"]=="SDFAStar":
                self.algos.append(("SDFAStar",SDFAStar(k["args"]["k1"],k["args"]["k2"])))
            if k["name"]=="WeightedAStar":
                self.algos.append(("WeightedAStar",WeightedAStar(k["args"]["weight"])))
            if k["name"]=="Dijkstra":
                self.algos.append(("Dijkstra",Dijkstra()))
            if k["name"]=="RRT":
                self.algos.append(("RRT",RRT(k["args"]["maxIter"],k["args"]["goalSampleRate"],k["args"]["stepSize"],k["args"]["pointSamples"])))
            if k["name"]=="CBFRRT":
                self.algos.append(("CBFRRT",CBFRRT(k["args"]["maxIter"],k["args"]["goalSampleRate"],k["args"]["stepSize"],k["args"]["pointSamples"],k["args"]["gamma1"],k["args"]["gamma2"])))
            if k["name"]=="VoronoiPlanner":
                self.algos.append(("VoronoiPlanner",VoronoiPlanner(k["args"]["pointSamples"],k["args"]["knn"])))
            if k["name"]=="UPP":
                self.algos.append(("UPP",UPP(k["args"]["alpha"],k["args"]["beta"],k["args"]["radius"],k["args"]["epsilon"])))
                
    def setUpEvals(self):
        for k in self.evalsDetails:
            if k["name"]=="SuccessRate":
                self.isSuccessEval=1
            if k["name"]=="PeakMemory":
                self.isPeakMemoryEval=1
            if k["name"]=="PlanningTime":
                self.isPlanningTimeEval=1
            if k["name"]=="PathCost":
                self.evals.append(("PathCost",PathCost()))
            if k["name"]=="NodesInPath":
                self.evals.append(("NodesInPath",NodesInPath(k["args"]["type"],k["args"]["epsilon"])))
            if k["name"]=="OptimalDeviation":
                self.evals.append(("OptimalDeviation",OptimalDeviation()))
            if k["name"]=="DistanceToGoal":
                self.evals.append(("DistanceToGoal",DistanceToGoal()))
            if k["name"]=="JerkPerMeter":
                self.evals.append(("JerkPerMeter",JerkPerMeter()))
            if k["name"]=="TurningAngle":
                self.evals.append(("TurningAngle",TurningAngle()))
            if k["name"]=="OptiSafeIndex":
                self.evals.append(("OptiSafeIndex",OptiSafeIndex(k["args"]["pointSamples"],k["args"]["knn"])))
            if k["name"]=="MinimumClearance":
                self.evals.append(("MinimumClearance",MinimumClearance(k["args"]["pointSamples"])))
            if k["name"]=="AverageMinimumClearance":
                self.evals.append(("AverageMinimumClearance",AverageMinimumClearance(k["args"]["pointSamples"])))
            if k["name"]=="ClearanceVariability":
                self.evals.append(("ClearanceVariability",ClearanceVariability(k["args"]["pointSamples"])))
            if k["name"]=="DangerViolations":
                self.evals.append(("DangerViolations",DangerViolations(k["args"]["pointSamples"],k["args"]["dangerRadius"])))
                           
    def setUpEnvs(self):
        for k in self.envsDetails:
            if "generateGrid" in k:
                envList= k["generateGrid"]
                for p in envList:
                    name="generateGrid_" + p["name"]
                    self.envs.append((name,GenerateGrid(p["name"])))
                    
    def getObjAndClass(self,data):
        name,obj=None,None
        for i in data:
            if type(i)==str:
                name=i
            else:
                obj=i
        
        return name,obj
    
    def iterationRanCheck(self,it,algoName):   
        iter="iter_"+ str(it)+".json"
        path=os.path.join(self.logger.runPath,algoName,iter)
        
        if os.path.exists(path):
            return True  
        
        
        return False
    
    def benchmark(self):
       
        for i in self.envs:
            name,obj=self.getObjAndClass(i)
            scenerio=obj.getmap()
            (grid, cellSize, envName,envDes, startGoalPairs)=scenerio
            for pair in startGoalPairs:
                self.iteration=self.iteration+1
                for algo in self.algos:
                    algoName,algoObj=self.getObjAndClass(algo)
                    evalData={}
                    if  not self.iterationRanCheck(self.iteration,algoName):
                        start = time.perf_counter()
                        pathData=algoObj.plan(pair["start"],pair["goal"],grid)
                        end   = time.perf_counter()
                        if self.isPeakMemoryEval:
                            tracemalloc.start()
                            pathData=algoObj.plan(pair["start"],pair["goal"],grid)
                            _, peak = tracemalloc.get_traced_memory()
                            tracemalloc.stop()
                            
                            PeakMemory= (peak/1024)
                            evalData["PeakMemory"]=PeakMemory
                    
                        diff=(end-start)*1000
                        
                        
                        for eval in self.evals:
                            
                            nameEval,objEval=self.getObjAndClass(eval)
                            
                            eval=objEval.eval(pair["start"],pair["goal"],grid,cellSize,pathData[1])
                            evalData[nameEval]=eval
                                
    
                        if self.isSuccessEval:
                            evalData["SuccessRate"]=pathData[0]  
                        if self.isPlanningTimeEval:      
                            evalData["PlanningTime"]=diff   
                            
                        
                        import gc; gc.collect()
                        self.logger.log(self.iteration,algoName,pathData,evalData,pair,scenerio)
                        del pathData, evalData

        
        print("Run Completed")
            
        
        
        
        
        
        
sf=SafePlan("/home/jatinarora/code/safeplan/safeplan/runs/run1.json")
sf.benchmark()