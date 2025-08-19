
import json

from algos.a_star import A_star



from envs.generate_grid import GenerateGrid


from evals.path_cost import PathCost

class SafePlan:
    def __init__(self,runConfigPath):
        self.runConfigPath=runConfigPath
        
        with open(self.runConfigPath) as file:
            data = json.load(file)
            
        self.envsDetails=data["envDetails"]
        self.algosDetails=data["algoDetails"]
        self.evalsDetails=data["evalDetails"]
        
        # classes objects lists
        self.envs=[]
        self.algos=[]
        self.evals=[]
        self.scenerios=[]

        
        self.setUpAlgos()
        self.setUpEvals()
        self.setUpEnvs()
        
        
    def setUpAlgos(self):
        for k in self.algosDetails:
            if k["name"]=="A_star":
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
    
    def benchmark(self):
       
        for i in self.envs:
            name,obj=self.getObjAndClass(i)
            data=obj.getmap()
            self.scenerios.append(data)
        
        for scenerio in self.scenerios:
            (grid, cellSize, envName, startGoalPairs)=scenerio
            for pair in startGoalPairs:
                for algo in self.algos:
                    name,obj=self.getObjAndClass(algo)
                    pathData=obj.plan(pair["start"],pair["goal"],grid)
                    for eval in self.evals:
                        name,obj=self.getObjAndClass(eval)
                        eval_data=obj.eval(pair["start"],pair["goal"],grid,cellSize,pathData[1])
                        print(eval_data)
                
            
        
        
        
        
        
        
sf=SafePlan("/home/jatinarora/code/safeplan/safeplan/runs/run1.json")
sf.benchmark()