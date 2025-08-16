from baseenv import BaseEnv
import json
import random
from itertools import combinations
from matplotlib.path import Path
from itertools import product
import numpy as np
from scipy.spatial import ConvexHull
import io
import os
"""
Reference for json file check:https://stackoverflow.com/questions/32991069/python-checking-for-json-files-and-creating-one-if-needed
    
"""


class GenerateGrid(BaseEnv):
    def __init__(self,configFolder):
        self.path="../safeplan/safeplan/configs/generate_grid"
        self.grid_suffix="_grid"
        self.configFolder=configFolder
        self.configName = self.path +"/" +self.configFolder  +"/"+ self.configFolder
        print(self.configName)
        self.finalGrid= self.configName + self.grid_suffix +".json"
        print(self.finalGrid) 
        self.grid=None
        self.A=[]
        self.b=[]
        

        
        
    def computeHalfspace(self):
        for k in range(len(self.polygons_data)):
            vertices= self.polygons_data[k]["polygon"]
            A,b = self.makeConvexHull(vertices) 
            self.A.append(A)
            self.b.append(b)
            
    def getRandomFreeCell(self):
        while True:
            cell=[]
            for k in range(self.dimension):
                cell.append(random.randrange(self.gridSize))
            
            cell=tuple(cell)
            if self.grid[cell] == 0:
                return cell
        
    def loadJson(self):
        with open(self.configName + ".json") as file:
            data = json.load(file)
        self.gridSize=data["gridSize"]
        self.envName=data["envName"]
        self.envDes=data["description"]
        self.dimension=data["dimensionOfGrid"]
        self.cellSize=data["cellSize"]
        self.randomStartGoal=data["randomStartGoal"]
        self.polygons_data=data["polygons"]
        self.numStartGoals=data["numStartGoals"]
        self.startGoalPairs=data["startGoalPairs"]
        
    
        
        
    def makeConvexHull(self,vertices):
        hull=ConvexHull(vertices)
        A = hull.equations[:, :-1]
        b = -hull.equations[:, -1]
        return (A,b)

    def inPolygons(self, x, eps=1e-3):
   
        x = np.asarray(x, float)
        for k in range(len(self.A)):
            if np.all(self.A[k] @ x <= self.b[k] + eps):
                return True
            
        return False
        
    def getmap(self):
        
        
        if os.path.isfile(self.finalGrid) and os.access(self.finalGrid, os.R_OK):
        # checks if file exists
            print ("File exists and is readable")
            with open(self.finalGrid) as file:
                load = json.load(file)
            self.grid=np.array(load["Grid"])
    
            self.envName=load["envName"]
            self.envDes=load["description"]
            self.dimension=load["dimensionOfGrid"]
            self.cellSize=load["cellSize"]
            self.startGoalPairs=load["startGoalPairs"]
            self.numStartGoals=load["numStartGoals"]
    
        else:
            print ("Either file is missing or is not readable, creating file...")
            writeJson={}
            
            self.loadJson()
        
            arraySize=[]
            for i in range(self.dimension):
                arraySize.append(self.gridSize)
        
            self.grid=np.zeros(tuple(arraySize))
            self.computeHalfspace()
        
            for k in product(range(self.gridSize), repeat=self.dimension):
                if self.inPolygons(k):
                    self.grid[k]=1
                
                    
            for k in range(len(self.polygons_data)):
                vertices= self.polygons_data[k]["polygon"]
                for coord in vertices:
                    self.grid[tuple(coord)]=1
            
            if self.randomStartGoal:
                self.startGoalPairs=[]
                k=0
                while k  < self.numStartGoals:
                    pair={}
                    start=self.getRandomFreeCell()
                    goal=self.getRandomFreeCell()
                    if start != goal:
                        k=k+1
                        
                        pair["start"]=start
                        pair["goal"]=goal
                        print(start)
                        self.startGoalPairs.append(pair)
                        
                print(self.startGoalPairs)
            writeJson["startGoalPairs"]=self.startGoalPairs
                    
            # Writing to json
            writeJson["Grid"]=self.grid.tolist()
            writeJson["envName"]=self.envName
            writeJson["description"]=self.envDes
            writeJson["dimensionOfGrid"]=self.dimension
            writeJson["cellSize"]=self.cellSize
            writeJson["numStartGoals"]=self.numStartGoals
            with open(self.finalGrid, "w") as f:
                    json.dump(writeJson, f)
            
        
        return (self.grid,self.cellSize,self.envName,self.startGoalPairs)
     
     
     
gr=GenerateGrid("env2")
gr.getmap()
    