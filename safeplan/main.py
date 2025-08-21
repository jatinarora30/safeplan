
import json
import os
import time

from .core.logger import Logger
from .algos.a_star import A_star





from .envs.generate_grid import GenerateGrid



from .evals.path_cost import PathCost

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
            if k["name"]=="A_Star":
                self.algos.append({"A_Star",A_star()})
                
    def setUpEvals(self):
        for k in self.evalsDetails:
            if k["name"]=="PathCost":
                self.evals.append({"PathCost",PathCost()})
        
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
                        evalData["Time"]=diff
                        evalData["Success"]=pathData[0]
                        for eval in self.evals:
                            nameEval,objEval=self.getObjAndClass(eval)
                            eval=objEval.eval(pair["start"],pair["goal"],grid,cellSize,pathData[1])
                            evalData[nameEval]=eval
                        
                        import gc; gc.collect()
                        self.logger.log(self.iteration,algoName,pathData,evalData,pair,scenerio)
                        del pathData, evalData
        
        print("Run Completed")
            
        
        
        
        
        
        
sf=SafePlan("/home/jatinarora/code/safeplan/safeplan/runs/run1.json")
sf.benchmark()