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
        self.evals.append("Success")
        self.evals.append("Time")
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
        self.totalFiles=test
        return True
    
    def compute(self):
        if self.checkValuesSame(self.iterations):
            print("same")
            
            meanAlgoData={}
            
            for algo in self.algos:
                path = os.path.join(self.runPath, algo)
                mean={}
                for k in self.evals:
                    mean[str(k)]=0
                for _, _, files in os.walk(path):
                    
                    for file in files:
                        filePath=os.path.join(path,file)
                        with open(filePath) as f:
                             data = json.load(f)
                             
                                 
                             for k in list(data["evaluation"].keys()):
                                mean[k]+=data["evaluation"][k]
                                
                mean = {key: value / self.totalFiles for key, value in mean.items()}
                mean["Success"]=mean["Success"]*100
                meanAlgoData[algo]=mean
            print(meanAlgoData)
        else:
            print("Values in all algorithms not same in iterations , can't compare")
        
        
            
    

            
        
        
        
        
sf=Stats("/home/jatinarora/code/safeplan/safeplan/runs/run1.json")
sf.compute()