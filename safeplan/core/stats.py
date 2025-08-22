"""
@file stats.py
@brief  Handles stats calculation of mean values of algo over all iterations in results folder
"""

import matplotlib.pyplot as plt
import pandas as pd
from pandas.plotting import table
import json
import os

class Stats:
    """
    A class to compute, compare, and visualize evaluation statistics for different algorithms
    based on run configurations and output data in JSON format.
    """
    
    def __init__(self, runConfigPath):
        """
        Initializes the Stats object by parsing the run configuration JSON file,
        creating necessary directories, and collecting details about algorithms and evaluations.
        
        Args:
            runConfigPath (str): Path to the run configuration JSON file.
        """
        self.runConfigPath = runConfigPath
        
        with open(self.runConfigPath) as file:
            data = json.load(file)
        
        self.outputDir = "outputs"
        self.algosDetails = data["algoDetails"]
        self.evalsDetails = data["evalDetails"]
        self.runDetails = data["runDetails"]
        self.resultsDir = "results"
        self.createDir(self.resultsDir)
        self.iterations = 0
        
        self.algos = []
        self.evals = []
        self.iterations = {}
        
        for k in self.algosDetails:
            self.algos.append(k["name"])
            self.iterations[k["name"]] = 0
        
        for k in self.evalsDetails:
            self.evals.append(k["name"])
        self.evals.append("Success")
        self.evals.append("Time")

        self.outputDir = "outputs"
        self.runPath = os.path.join(self.outputDir, self.runDetails)
        
        # Count the number of iterations (files) for each algorithm
        for algo in self.algos:
            path = os.path.join(self.runPath, algo)
            self.iterations[algo] = sum(len(files) for _, _, files in os.walk(path))

    def createDir(self, path):
        """
        Safely creates a directory if it does not already exist.

        Args:
            path (str): The path of the directory to create.
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
            
    def checkValuesSame(self, iterations):
        """
        Checks if all algorithms have the same number of iterations.

        Args:
            iterations (dict): Dictionary with algorithm names as keys and iteration counts as values.

        Returns:
            bool: True if all values are the same, False otherwise.
        """
        test = list(iterations.values())[0]
        for val in list(iterations.values()):
            if val != test:
                return False
        self.totalFiles = test
        return True
    
    def compute(self):
        """
        Computes the average evaluation metrics across all iterations for each algorithm.
        Saves the results as a table image and a JSON file in the results directory.
        """
        if self.checkValuesSame(self.iterations):
            
            meanAlgoData = {}
            
            for algo in self.algos:
                path = os.path.join(self.runPath, algo)
                mean = {}
                for k in self.evals:
                    mean[str(k)] = 0

                # Aggregate values across all iteration files
                for _, _, files in os.walk(path):
                    for file in files:
                        filePath = os.path.join(path, file)
                        with open(filePath) as f:
                            data = json.load(f)
                            for k in list(data["evaluation"].keys()):
                                mean[k] += data["evaluation"][k]
                
                # Compute mean values
                mean = {key: value / self.totalFiles for key, value in mean.items()}
                mean["Success"] = mean["Success"] * 100  # Convert to percentage
                meanAlgoData[algo] = mean

                # Convert to DataFrame for display
                df = pd.DataFrame(meanAlgoData).T

            print(df)

            # Plot as a table
            ax = plt.subplot(111, frame_on=False)
            ax.xaxis.set_visible(False)
            ax.yaxis.set_visible(False)
            table(ax, df, loc="center")
            plt.savefig(self.resultsDir + "/" + self.runDetails + ".png")

            # Save JSON summary
            meanAlgoData = json.dumps(meanAlgoData, indent=4)
            with open(os.path.join(self.resultsDir, self.runDetails + "Stats.json"), "w") as f:
                f.write(meanAlgoData)
        else:
            print("Values in all algorithms not same in iterations, can't compare")

        
