
import json
import os
import time

from .core.logger import Logger
from .algos.a_star import AStar





from .envs.generate_grid import GenerateGrid




from .evals.path_cost import PathCost
from .evals.nodes_in_path import NodesInPath
from .evals.optimal_deviation import OptimalDeviation
from .evals.distance_to_goal import DistanceToGoal
from .evals.jerk_per_meter import JerkPerMeter

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
        
        # classes objects lists
        self.envs=[]
        self.algos=[]
        self.evals=[]
        self.scenerios=[]
        

        
        self.setUpAlgos()
        self.setUpEvals()
        self.setUpEnvs()
        print(self.evals)
        
        self.logger = Logger(self.runDetails,self.algos,self.outputDir)
        self.runPath=os.path.join(self.outputDir,self.runDetails)
        
        
        
    def setUpAlgos(self):
        for k in self.algosDetails:
            if k["name"]=="AStar":
                self.algos.append({"AStar",AStar()})
                
    def setUpEvals(self):
        for k in self.evalsDetails:
            if k["name"]=="SuccessRate":
                self.isSuccessEval=1
            if k["name"]=="PlanningTime":
                self.isPlanningTimeEval=1
            if k["name"]=="PathCost":
                self.evals.append({"PathCost",PathCost()})
            if k["name"]=="NodesInPath":
                self.evals.append({"NodesInPath",NodesInPath(k["args"]["type"],k["args"]["epsilon"])})
            if k["name"]=="OptimalDeviation":
                self.evals.append({"OptimalDeviation",OptimalDeviation()})
            if k["name"]=="DistanceToGoal":
                self.evals.append({"DistanceToGoal",DistanceToGoal()})
            if k["name"]=="JerkPerMeter":
                self.evals.append({"JerkPerMeter",JerkPerMeter()})
    def setUpEnvs(self):
        for k in self.envsDetails:
            if "generateGrid" in k:
                envList= k["generateGrid"]
                for p in envList:
                    name="generateGrid_" + p["name"]
                    self.envs.append({name,GenerateGrid(p["name"])})
                    
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
                    if  not self.iterationRanCheck(self.iteration,algoName):
                        start = time.perf_counter()
                        pathData=algoObj.plan(pair["start"],pair["goal"],grid)
                        end   = time.perf_counter()
                        diff=(end-start)*1000
                        evalData={}
                        
                        for eval in self.evals:
                            
                            nameEval,objEval=self.getObjAndClass(eval)
                            
                            eval=objEval.eval(pair["start"],pair["goal"],grid,cellSize,pathData[1])
                            evalData[nameEval]=eval
                            print(self.evalsDetails)
                                
    
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