"""
@file generate_grid.py
@brief Utilities to generate N-dimensional occupancy grids with polygonal obstacles and optional random start/goal pairs.
@details
  Reads a scene configuration, rasterizes polygon obstacles into a discrete grid,
  and (optionally) samples random free start/goal pairs. Caches the generated
  grid to a JSON file for quick reloads.

"""

from .baseenv import BaseEnv
import json
import random
from itertools import product
import numpy as np
from scipy.spatial import ConvexHull
import os


class GenerateGrid(BaseEnv):
    """
    @class GenerateGrid
    @brief Generates and caches a discrete occupancy grid with polygonal obstacles.
    @details
      The class loads a scene configuration (grid size, polygons, etc.),
      computes half-space representations for polygons, rasterizes them
      into a grid (1 = occupied, 0 = free), and optionally samples random
      start/goal pairs lying in free cells. The generated grid is cached
      to <configName>_grid.json for repeatability and faster subsequent runs.

    @note
      - Grid indexing is 0-based across all dimensions.
      - Obstacles are considered "inside" if a cell index (interpreted as a point)
        satisfies all half-space inequalities for at least one polygon.
      - Vertices are explicitly marked as occupied.

    @inherit BaseEnv
    """

    def __init__(self, configFolder: str):
        """
        @brief Constructor.
        @param configFolder Name of the configuration subfolder under
               ../safeplan/safeplan/configs/generate_grid/ that contains
               the base JSON (without the _grid suffix).
        @post
          - self.finalGrid points to the output JSON cache path.
          - Internal containers (grid, A, b) are initialized.
        """
        self.path = "../safeplan/safeplan/configs/generate_grid"
        self.grid_suffix = "_grid"
        self.configFolder = configFolder
        self.configName = self.path + "/" + self.configFolder + "/" + self.configFolder
        self.finalGrid = self.configName + self.grid_suffix + ".json"
        self.grid = None
        self.A = []  # List[np.ndarray]: half-space A matrices for each polygon
        self.b = []  # List[np.ndarray]: half-space b vectors for each polygon

    def computeHalfspace(self) -> None:
        """
        @brief Compute half-space representations (A, b) for all polygons.
        @details
          Uses @c scipy.spatial.ConvexHull to obtain the H-representation
          of each polygon from its vertices. Stores the results into
          @c self.A and @c self.b as parallel lists.
        @pre self.polygons_data must be loaded via loadJson().
        @post self.A[k], self.b[k] hold the half-space for polygon k.
        """
        for k in range(len(self.polygons_data)):
            vertices = self.polygons_data[k]["polygon"]
            A, b = self.makeConvexHull(vertices)
            self.A.append(A)
            self.b.append(b)

    def getRandomFreeCell(self) -> tuple:
        """
        @brief Sample a random free cell uniformly from the grid.
        @return A tuple of indices (one per dimension) for a free cell (value == 0).
        @throws RuntimeError If @c self.grid is not initialized.
        @note This loops until a free cell is found; for heavily occupied maps
              this may take longer.
        """
        if self.grid is None:
            raise RuntimeError("Grid not initialized. Call getmap() after loadJson().")

        while True:
            cell = []
            for k in range(self.dimension):
                cell.append(random.randrange(self.gridSize))
            cell = tuple(cell)
            if self.grid[cell] == 0:
                return cell

    def loadJson(self) -> None:
        """
        @brief Load the base environment configuration.
        @details
          Reads @c <configName>.json and populates:
          - gridSize, envName, envDes, dimension, cellSize,
            randomStartGoal, polygons_data, numStartGoals, startGoalPairs.
        @throws FileNotFoundError If the base configuration file does not exist.
        """
        with open(self.configName + ".json") as file:
            data = json.load(file)
        self.gridSize = data["gridSize"]
        self.envName = data["envName"]
        self.envDes = data["envDes"]
        self.dimension = data["dimensionOfGrid"]
        self.cellSize = data["cellSize"]
        self.randomStartGoal = data["randomStartGoal"]
        self.polygons_data = data["polygons"]
        self.numStartGoals = data["numStartGoals"]
        self.startGoalPairs = data["startGoalPairs"]

    def makeConvexHull(self, vertices) -> tuple:
        """
        @brief Convert a set of polygon vertices to half-space form.
        @param vertices Iterable of vertex coordinates (shape: N x D).
        @return (A, b) where A @ x <= b describes the convex hull.
        @details
          Uses @c scipy.spatial.ConvexHull to compute plane equations in the
          form [A | c] so that A x + c = 0 for faces. We rearrange to A x <= b
          with b = -c.
        @warning Vertices must define a valid (non-degenerate) convex polygon/polytope.
        """
        hull = ConvexHull(vertices)
        A = hull.equations[:, :-1]
        b = -hull.equations[:, -1]
        return (A, b)

    def inPolygons(self, x, eps: float = 1e-3) -> bool:
        """
        @brief Test if a point lies inside any polygon (inclusive, with tolerance).
        @param x Point to test; will be cast to float ndarray (shape: D,).
        @param eps Numerical tolerance added to @c b when checking A @ x <= b + eps.
        @return True if x is inside at least one polygon; False otherwise.
        @pre self.A and self.b computed by computeHalfspace().
        """
        x = np.asarray(x, float)
        for k in range(len(self.A)):
            if np.all(self.A[k] @ x <= self.b[k] + eps):
                return True
        return False

    def getmap(self):
        """
        @brief Build or load the occupancy grid and associated metadata.
        @details
          - If a cached grid file (<configName>_grid.json) exists, load it.
          - Else, load base config, build an N-D zero grid, mark obstacle
            cells by polygon half-space tests, and (optionally) generate
            random start/goal pairs in free cells. Then cache to JSON.

        @return Tuple (grid, cellSize, envName, startGoalPairs):
          - grid (np.ndarray): N-D array of 0 (free) / 1 (occupied).
          - cellSize (float/int): Size of a cell in world units.
          - envName (str): Environment name.
          - startGoalPairs (list): List of dicts with "start" and "goal" tuples.

        @throws FileNotFoundError If neither the cache nor the base config exists.
        @post
          - self.grid is a numpy array with obstacles marked as 1.
          - self.startGoalPairs is populated (from config or randomly).
          - A JSON cache is written when generating from base config.
        """
        if os.path.isfile(self.finalGrid) and os.access(self.finalGrid, os.R_OK):
            # Cached grid exists; load it.
            print("File exists and is readable")
            with open(self.finalGrid) as file:
                load = json.load(file)
            self.grid = np.array(load["Grid"])

            self.envName = load["envName"]
            self.envDes = load["envDes"]
            self.dimension = load["dimensionOfGrid"]
            self.cellSize = load["cellSize"]
            self.startGoalPairs = load["startGoalPairs"]
            self.numStartGoals = load["numStartGoals"]

        else:
            print("Either file is missing or is not readable, creating file...")
            writeJson = {}

            # 1) Load base config
            self.loadJson()

            # 2) Allocate grid
            arraySize = []
            for i in range(self.dimension):
                arraySize.append(self.gridSize)
            self.grid = np.zeros(tuple(arraySize))

            # 3) Rasterize polygons via half-space inclusion
            self.computeHalfspace()
            for k in product(range(self.gridSize), repeat=self.dimension):
                if self.inPolygons(k):
                    self.grid[k] = 1

            # 4) Mark vertices explicitly as occupied
            for k in range(len(self.polygons_data)):
                vertices = self.polygons_data[k]["polygon"]
                for coord in vertices:
                    self.grid[tuple(coord)] = 1

            # 5) Random start/goal pairs, if requested
            if self.randomStartGoal:
                self.startGoalPairs = []
                k = 0
                while k < self.numStartGoals:
                    pair = {}
                    start = self.getRandomFreeCell()
                    goal = self.getRandomFreeCell()
                    if start != goal:
                        k = k + 1
                        pair["start"] = start
                        pair["goal"] = goal
                        self.startGoalPairs.append(pair)
            writeJson["startGoalPairs"] = self.startGoalPairs

            # 6) Write cache JSON
            writeJson["Grid"] = self.grid.tolist()
            writeJson["envName"] = self.envName
            writeJson["envDes"] = self.envDes
            writeJson["dimensionOfGrid"] = self.dimension
            writeJson["cellSize"] = self.cellSize
            writeJson["numStartGoals"] = self.numStartGoals
            jsonStr = json.dumps(writeJson, indent=4)
            with open(self.finalGrid, "w") as f:
                f.write(jsonStr)

        return (self.grid, self.cellSize, self.envName,self.envDes, self.startGoalPairs)


    