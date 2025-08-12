"""
Reference for A star: https://www.geeksforgeeks.org/python/a-search-algorithm-in-python/ 

Reference for adjacent nodes: https://www.geeksforgeeks.org/python/python-adjacent-coordinates-in-n-dimension/ 
"""

from baseplanner import BasePlanner
import math 
import heapq
import itertools
import numpy as np

class A_star(BasePlanner):
    
    def __init__(self):
        print("Intializing A* ...")
        
        self.success=0
        self.info=[]
        
        self.path= []
     
        
    def isValid(self, grid_cell):
        for i in range(self.dimension):
            if not (0 <= grid_cell[i] < self.grid.shape[i]):
                return False
        return True

    def heuristics(self,node1,node2):
        cost=0
        
        for i in range(0,self.dimension):
            cost+=np.square(abs(node1[i]-node2[i]))
        return np.sqrt(cost)
            
            
    def adjacentCoordinates(self,node):
        offsets=[-1,0,1]
        combinations=itertools.product(offsets,repeat=self.dimension)
        adjacentNodes=[]
        for c in combinations:
            if all (o ==0 for o in c):
                continue
            nb= tuple(node[i]+c[i] for i in range(self.dimension))
            adjacentNodes.append(nb)
        return adjacentNodes
                
        
    def plan(self,start,goal,grid):
        self.start=tuple(start)
        self.goal=tuple(goal)
        self.grid=grid
        self.path =[]
        self.info = []
        self.success=0
        self.dimension=len(start)
        self.sizes=[]
        for i in range(0,self.dimension):
            self.sizes.append(np.size(grid,axis=i))
        print(self.sizes)
        if not self.isValid(self.start):
            self.info.append("Invalid start ")
            return self.success,self.path,self.info
        
        if not self.isValid(self.goal):
            self.info.append("Invalid  goal")
            return self.success,self.path,self.info
        
        if self.grid[self.goal]==1:
            self.info.append("Goal has obstacle")
            return self.success,self.path,self.info
        
        if self.grid[self.start]==1:
            self.info.append("Start has obstacle")
            return self.success,self.path,self.info
        
        if self.start==self.goal:
            self.info.append("Start and goal are same")
            self.success = 1
            self.path = [self.start]
            return self.success,self.path,self.info
        
        heap=[]
        visited=set()
        parents={}
        f0=self.heuristics(self.start,self.goal)
        heapq.heappush(heap,(f0,0,self.start))
        self.parent=None
        self.node=self.start
        
        while heap:
            _,g,self.node=heapq.heappop(heap)
            visited.add(self.node)
        
            
            #calculating alternate nodes
            adjacentNodes=self.adjacentCoordinates(self.node)
            print(adjacentNodes)
            print
            
            for k in range(len(adjacentNodes)):
                if self.isValid(adjacentNodes[k]) and self.grid[adjacentNodes[k]]==0 and adjacentNodes[k] not in visited:
                    self.parent=self.node
                    if adjacentNodes[k]==self.goal:
                        
                        parents[adjacentNodes[k]]=self.parent
                        self.success=1
                    else:
                        visited.add(adjacentNodes[k])
                        parents[adjacentNodes[k]]=self.parent
                        g_updated=g+self.heuristics(self.node,adjacentNodes[k])
                        total_cost=self.heuristics(self.node,self.goal)+g_updated
                        heapq.heappush(heap,(total_cost,g_updated,adjacentNodes[k]))
                        
                        
        if self.success==1:
            path_node=self.goal
            while path_node!=self.start:
                self.path.append(path_node)
                path_node=parents.get(path_node)
                
            self.path.append(self.start)
            self.path.reverse()
            print(type(self.path))
            
        
        return self.success,self.path,self.info
            
                       
        



tests = [
    # (start, goal, grid, description)

    # T1: Empty grid, straight line
    ([0, 0], [0, 4], np.zeros((5, 5), dtype=int), "Empty grid, straight path"),

    # T2: Start == Goal
    ([1, 1], [1, 1], np.zeros((3, 3), dtype=int), "Start equals goal"),

    # T3: No path (solid wall)
    ([1, 2], [3, 2], (lambda g: (g.__setitem__((2, slice(None)), 1), g)[1])(np.zeros((5, 5), dtype=int)),
     "No path due to wall"),

    # T4: Goal cell blocked
    ([0, 0], [2, 2], (lambda g: (g.__setitem__((2, 2), 1), g)[1])(np.zeros((3, 3), dtype=int)),
     "Goal is an obstacle"),

    # T5: Start cell blocked
    ([0, 0], [2, 2], (lambda g: (g.__setitem__((0, 0), 1), g)[1])(np.zeros((3, 3), dtype=int)),
     "Start is an obstacle"),

    # T6: Your original maze-like grid
    ([6, 4], [6, 2], np.array([
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1, 0, 1, 0, 1],
        [0, 0, 1, 0, 1, 0, 0, 0, 0, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
        [1, 0, 1, 1, 1, 1, 0, 1, 0, 0],
        [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 0, 0, 1, 0, 0, 1]
    ], dtype=int), "Given example"),

    # T7: Narrow corridor with a turn
    ([1, 1], [4, 4], (lambda g: (g.__setitem__((slice(1,5), 1), 0),
                                  g.__setitem__((4, slice(1,5)), 0), g)[2])(np.ones((6, 6), dtype=int)),
     "Narrow corridor with turn"),

    # T8: Multiple shortest paths
    ([0, 0], [3, 3], np.zeros((4, 4), dtype=int), "Multiple optimal paths"),

    # T9: Diagonal-only path (succeeds only if diagonals allowed)
    ([0, 0], [1, 1], np.array([[0, 1],
                                [1, 0]], dtype=int), "Diagonal-only path"),

    # T10: Invalid start (outside grid)
    ([-1, 0], [2, 2], np.zeros((3, 3), dtype=int), "Invalid start index"),

    # T11: Invalid goal (outside grid)
    ([0, 0], [3, 3], np.zeros((3, 3), dtype=int), "Invalid goal index"),

    # T12: 3D grid with obstacles (N-D support)
    ([0, 0, 0], [2, 2, 2], (lambda g: (
        g.__setitem__((1, 1, slice(None)), 1),
        g.__setitem__((1, slice(None), 1), 1),
        g.__setitem__((slice(None), 1, 1), 1), g)[3])(np.zeros((3, 3, 3), dtype=int)),
     "3D grid with central cross blocked"),
]

# --- Harness ---
planner = A_star()
for i, (s, t, g, desc) in enumerate(tests, 1):
    print(f"\n=== Test {i}: {desc} ===")
    success, path, info = planner.plan(s, t, g)
    print("Success:", success)
    print("Path   :", path)
    print("Info   :", info)
