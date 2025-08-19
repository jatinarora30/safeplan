import os
import json
class Logger:
    
    def __init__(self,runDetails,algos):
        self.algos=algos
        self.runDetails=runDetails
        self.outputDir="outputs"
        self.createDir(self.outputDir)   
        self.runPath=os.path.join("outputs",self.runDetails)
        self.createDir(self.runPath)
        for algo in self.algos:
            name,_=self.getObjAndClass(algo)
            path=os.path.join(self.runPath,name)
            self.createDir(path)

    def getObjAndClass(self,data):
        name,obj=None,None
        for i in data:
            if type(i)==str:
                name=i
            else:
                obj=i
        
        return name,obj
        
    def createDir(self,path):
        try:
            os.mkdir(path)
            print(f"Directory '{path}' created successfully.")
        except FileExistsError:
            print(f"Directory '{path}' already exists.")
        except PermissionError:
            print(f"Permission denied: Unable to create '{path}'.")
        except Exception as e:
            print(f"An error occurred: {e}")
            
            
    def log(self,iter,algo,pathData,evalData,pair,scenerio):
        filename="iter_"+str(iter)+".json"
        path=os.path.join(self.runPath,algo,filename)
        jsonData={}
        pathData=list(pathData)
        jsonData["evaluation"]=evalData
        jsonData["startGoal"]=pair
        
        pathInfo={}
        pathInfo["success"]=pathData[0]
        pathInfo["path"]=pathData[1]
        pathInfo["info"]=pathData[2]
        jsonData["pathInfo"]=pathInfo
        
        envInfo={}
        scenerio=list(scenerio)
        envInfo["grid"]=scenerio[0].tolist()
        envInfo["cellSize"]=scenerio[1]
        envInfo["envName"]=scenerio[2]
        envInfo["envDes"]=scenerio[3]
        jsonData["envInfo"]=envInfo
        
        print(jsonData)
        jsonStr = json.dumps(jsonData, indent=4)
        with open(path, "w") as f:
            f.write(jsonStr)
        print(scenerio,pair)
        
        
        