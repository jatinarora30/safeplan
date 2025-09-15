import matplotlib.pyplot as plt
import pandas as pd
from pandas.plotting import table
import json
import os

class Stats:
    """
    Compute, compare, and visualize evaluation statistics for different algorithms
    based on run configurations and output data in JSON format.
    """
    def __init__(self, runConfigPath, saveStatsImage=False):
        """
        Args:
            runConfigPath (str): Path to the run configuration JSON file.
            saveStatsImage (bool): Whether to save the stats as a table image.
        """
        self.runConfigPath = runConfigPath
        self.saveStatsImage = saveStatsImage

        with open(self.runConfigPath) as file:
            data = json.load(file)

        self.resultsDir = "results"
        self.createDir(self.resultsDir)

        # Expecting these keys in the config JSON
        self.algosDetails = data["algoDetails"]
        self.evalsDetails = data["evalDetails"]
        self.runDetails = data["runDetails"]  # e.g., a run name like "exp_01"

        # Prepare names
        self.algos = [k["name"] for k in self.algosDetails]

        # Avoid duplicates in eval names
        self.evals = []
        for k in self.evalsDetails:
            name = k["name"]
            if name not in self.evals:
                self.evals.append(name)

        # Where outputs live: outputs/<runDetails>/<algo>/*.json
        self.outputDir = "outputs"
        self.runPath = os.path.join(self.outputDir, self.runDetails)

        # Count iterations per algo
        self.iterations = {}
        for algo in self.algos:
            path = os.path.join(self.runPath, algo)
            count = 0
            for _, _, files in os.walk(path):
                count += len([f for f in files if f.lower().endswith(".json")])
            self.iterations[algo] = count

    def createDir(self, path):
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
        Returns True if all algorithms have the same number of iterations.
        Also sets self.totalFiles to that common value when True.
        """
        vals = list(iterations.values())
        if not vals:
            return False
        test = vals[0]
        for v in vals:
            if v != test:
                return False
        self.totalFiles = test
        return True

    def compute(self):
        """
        Computes the average evaluation metrics across all iterations for each algorithm.
        Saves the results as a table image (optional) and a JSON file in the results directory.
        """
        meanAlgoData = {}
        for algo in self.algos:
            path = os.path.join(self.runPath, algo)

            # Accumulators
            sums = {k: 0.0 for k in self.evals}
            n = 0

            # Walk through all JSON result files for this algo
            for _, _, files in os.walk(path):
                for fname in files:
                    if not fname.lower().endswith(".json"):
                        continue
                    fpath = os.path.join(path, fname)
                    try:
                        with open(fpath, "r") as f:
                            data = json.load(f)
                    except Exception as e:
                        print(f"Skipping '{fpath}': {e}")
                        continue

                    eval_block = data.get("evaluation", {})
                    # Sum only keys we care about; missing keys are treated as 0 (or skip if you prefer)
                    for k in self.evals:
                        if k in eval_block:
                            sums[k] += eval_block[k]
                    n += 1

            if n == 0:
                print(f"No result files found for algo '{algo}' at {path}")
                # Keep NaNs to signal missing data
                meanAlgoData[algo] = {k: float("nan") for k in self.evals}
            else:
                meanAlgoData[algo] = {k: (sums[k] / n) for k in self.evals}

        # Build DataFrame
        df = pd.DataFrame.from_dict(meanAlgoData, orient="index")[self.evals]  # preserve column order
        print(df)

        # Optional: save an image of the table
        if self.saveStatsImage:
            fig = plt.figure(figsize=(max(6, len(self.evals) * 1.2), max(3, len(self.algos) * 0.6)))
            ax = plt.subplot(111, frame_on=False)
            ax.xaxis.set_visible(False)
            ax.yaxis.set_visible(False)
            table(ax, df.round(4), loc="center")
            plt.tight_layout()
            out_png = os.path.join(self.resultsDir, f"{self.runDetails}.png")
            plt.savefig(out_png, dpi=200)
            plt.close(fig)

        # Save JSON summary
        out_json = os.path.join(self.resultsDir, f"{self.runDetails}Stats.json")
        with open(out_json, "w") as f:
            json.dump(meanAlgoData, f, indent=4)
