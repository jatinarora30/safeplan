"""
@file logger.py
@brief  Handles logging of path planning experiments including environment, evaluation, and path data
"""

import os
import json

class Logger:
    """
    @class Logger
    @brief Handles logging of path planning experiments including environment, evaluation, and path data.
    """

    def __init__(self, runDetails, algos, outputDir):
        """
        @brief Constructor for Logger class.

        @param runDetails A string specifying details about the run (used to create a subdirectory).
        @param algos A list of tuples where each tuple contains (algorithm_name: str, algorithm_object: Any).
        @param outputDir Path to the main output directory.
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
        @brief Extracts the name and object from a tuple containing a string and an object.

        @param data A list/tuple where one item is a string (name) and the other is an object.
        @return (name, obj) A tuple of the algorithm name and its corresponding object.
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
        @brief Safely creates a directory, handling common exceptions.

        @param path The path to the directory to create.
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
        @brief Logs path planning data into a structured JSON file.

        @param iter Iteration number of the current experiment run.
        @param algo Name of the algorithm used in this iteration.
        @param pathData A tuple/list with [success flag, path list, additional info dict].
        @param evalData Evaluation metrics 
        @param pair A tuple/list with start and goal coordinates.
        @param scenerio A tuple/list with [grid array, cell size, environment name, environment description].
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
        envInfo["grid"] = scenerio[0].tolist()
        envInfo["cellSize"] = scenerio[1]
        envInfo["envName"] = scenerio[2]
        envInfo["envDes"] = scenerio[3]
        jsonData["envInfo"] = envInfo

        jsonStr = json.dumps(jsonData, indent=4)
        with open(path, "w") as f:
            f.write(jsonStr)
