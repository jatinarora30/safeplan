"""
@file logger.py
@brief Log path-planning experiments (environment, evaluation, and path data) to JSON files.

@details
Creates a per-run directory structure and writes one JSON file per iteration and algorithm.
Each file captures:
- Algorithm name
- Evaluation metrics (as provided by the caller)
- Start/goal pair
- Path info (sequence, diagnostics)
- Environment info (grid, cell size, environment name and description)

Directory layout:
- <outputDir>/<runDetails>/<algo>/iter_<N>.json

@note
- Directories are created on demand with basic error handling.
- The grid is serialized via `tolist()` for JSON compatibility.
- The `success` flag (if present in @p pathData[0]) is not currently written (line kept commented).
"""

import os
import json

class Logger:
    """
    @class Logger
    @brief Handles logging of path planning experiments including environment, evaluation, and path data.

    @details
    Initializes a run directory and one subdirectory per algorithm, then provides
    a method to write a structured JSON for each iteration and algorithm.
    """

    def __init__(self, runDetails, algos, outputDir):
        """
        @brief Construct a Logger and prepare the output directory tree.

        @param runDetails str
               Identifier for the run (used to create a subdirectory under @p outputDir).
        @param algos list[tuple[str, Any]]
               Sequence of (algorithm_name, algorithm_object) tuples.
        @param outputDir str
               Root directory where run and algorithm subfolders will be created.

        @post
        - Ensures @p outputDir exists.
        - Creates @c <outputDir>/<runDetails>/ and one subdirectory per algorithm name.
        """
        self.algos = algos
        self.runDetails = runDetails
        self.outputDir = outputDir

        self.createDir(self.outputDir)
        self.runPath = os.path.join(outputDir, self.runDetails)
        self.createDir(self.runPath)

        for algo in self.algos:
            name, _ = self.getObjAndClass(algo)
            path = os.path.join(self.runPath, name)
            self.createDir(path)

    def getObjAndClass(self, data):
        """
        @brief Extract the algorithm name and object from a (name, object) pair.

        @param data tuple|list
               Container with exactly one string (name) and one object (algorithm instance).
        @return tuple[str, Any]
                (name, obj) extracted from @p data.
        """
        name, obj = None, None
        for i in data:
            if type(i) == str:
                name = i
            else:
                obj = i

        return name, obj

    def createDir(self, path):
        """
        @brief Create a directory if needed, with simple exception handling.

        @param path str
               Directory path to create.

        @return None
                Prints a message on success, existence, or failure.
        """
        try:
            os.mkdir(path)
            print(f"Directory '{path}' created successfully.")
        except FileExistsError:
            print(f"Directory '{path}' already exists.")
        except PermissionError:
            print(f"Permission denied: Unable to create '{path}'.")
        except Exception as e:
            print(f"An error occurred: {e}")

    def log(self, iter, algo, pathData, evalData, pair, scenerio):
        """
        @brief Write a single iteration’s results for one algorithm to JSON.

        @details
        Produces `iter_<iter>.json` in the algorithm’s subdirectory. The JSON contains:
        - `algo`       : algorithm name (str)
        - `evaluation` : evaluation metrics (as provided)
        - `startGoal`  : start/goal data (as provided)
        - `pathInfo`   : dict with `path` and `info` from @p pathData
        - `envInfo`    : dict with `grid` (as list), `cellSize`, `envName`, `envDes`

        @param iter int
               Iteration number.
        @param algo str
               Algorithm name (must match a subdirectory prepared in the constructor).
        @param pathData list|tuple
               [success_flag, path_list, info] — only `path_list` and `info` are written.
        @param evalData dict
               Evaluation metrics to record.
        @param pair list|tuple
               Start/goal container to store under `startGoal` (e.g., {"start": ..., "goal": ...}).
        @param scenerio list|tuple
               [grid_array, cell_size, env_name, env_description].

        @return None
                Writes the JSON file to disk.
        """
        filename = "iter_" + str(iter) + ".json"
        path = os.path.join(self.runPath, algo, filename)
        jsonData = {}

        pathData = list(pathData)
        jsonData["algo"] = algo
        jsonData["evaluation"] = evalData
        jsonData["startGoal"] = pair

        pathInfo = {}
        #pathInfo["success"] = pathData[0]
        pathInfo["path"] = pathData[1]
        pathInfo["info"] = pathData[2]
        jsonData["pathInfo"] = pathInfo

        envInfo = {}
        scenerio = list(scenerio)
       #  envInfo["grid"] = scenerio[0].tolist()
        envInfo["cellSize"] = scenerio[1]
        envInfo["envName"] = scenerio[2]
        envInfo["envDes"] = scenerio[3]
        jsonData["envInfo"] = envInfo

        jsonStr = json.dumps(jsonData, indent=4)
        with open(path, "w") as f:
            f.write(jsonStr)
