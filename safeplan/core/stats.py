import json
import os
class Stats:
    
    def __init__(self,runConfigPath):
        self.runConfigPath=runConfigPath
        
        with open(self.runConfigPath) as file:
            data = json.load(file)
        
        self.outputDir="outputs"
        self.algosDetails=data["algoDetails"]
        self.evalsDetails=data["evalDetails"]
        self.runDetails=data["runDetails"]
        self.iterations=0
        
        self.algos=[]
        self.evals=[]
        self.iterations={}
        
        
        for k in self.algosDetails:
            self.algos.append(k["name"])
            self.iterations[k["name"]]=0
        
        for k in self.evalsDetails:
            self.evals.append(k["name"])
        self.outputDir = "outputs"
        self.runPath = os.path.join(self.outputDir, self.runDetails)

        for algo in self.algos:
            path = os.path.join(self.runPath, algo)
            self.iterations[algo]=sum(len(files) for _, _, files in os.walk(path))
        
       
        
    def checkValuesSame(self,iterations):
        test=list(iterations.values())[0]
        for val in list(iterations.values()):
            if val!=test:
                return False
        
        return True
    
    def compute(self):
        if self.checkValuesSame(self.iterations):
            print("same")
        else:
            print("Values in all algorithms not same in iterations , can't compare")
        
        
            
    

            
        
        
        
        
sf=Stats("/home/jatinarora/code/safeplan/safeplan/runs/run1.json")
sf.compute()