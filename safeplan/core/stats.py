"""
@file stats.py
@brief Aggregate, compare, and visualize benchmark statistics from JSON outputs.

@details
Loads a run configuration (algorithms, evaluations, and run metadata), scans
per-algorithm JSON outputs produced under a run directory, computes averaged
metrics, and optionally renders a table figure. Results are saved as a JSON
summary (and PNG if requested).

Directory conventions:
- Outputs are expected under `outputs/<runDetails>/<algo>/*.json`.
- Aggregated stats are written to `results/<runDetails>Stats.json`.
- Optional figure is saved to `results/<runDetails>.png`.

Evaluation handling:
- The code builds a consistent evaluation key list (`self.evals`) from the
  config file and averages each metric across all JSON files found for a given
  algorithm.
- Keys matching a generic “success rate” pattern (e.g., `SuccessRate`,
  `success_rate`, `success`) are normalized by the **global** maximum iteration
  count across algorithms; all other metrics are averaged over the number of
  files found for that algorithm.

@par Expected Config Schema (JSON at @p runConfigPath)
- @c algoDetails : list of objects with at least a @c name field for each algorithm.
- @c evalDetails : list of objects with at least a @c name field for each metric.
- @c runDetails  : string run identifier (used as subdirectory under @c outputs/).

@par Outputs
- Printed pandas DataFrame of averaged metrics (row = algorithm, col = metric).
- JSON summary written to @c results/<runDetails>Stats.json.
- Optional PNG table at @c results/<runDetails>.png when @p saveStatsImage is True.

@note
- Missing or malformed result files are skipped with a console message.
- If an algorithm has no result files, its metrics are set to NaN in the output.
- The module creates a @c results/ directory if it does not already exist.

@see pandas.DataFrame, matplotlib, pandas.plotting.table
"""

import matplotlib.pyplot as plt
import pandas as pd
from pandas.plotting import table
import json
import os

class Stats:
    """
    @class Stats
    @brief Compute, compare, and visualize evaluation statistics across algorithms.

    @details
    Given a run configuration describing algorithms and evaluation metrics, this
    class scans the corresponding output folders, aggregates metrics per algorithm,
    and produces a tabular summary (printed, saved to JSON, and optionally saved
    as a PNG table image).
    """
    def __init__(self, runConfigPath, saveStatsImage=False):
        """
        @brief Initialize the statistics pipeline from a run configuration.

        @param runConfigPath str
               Path to the run configuration JSON file.
        @param saveStatsImage bool
               Whether to save a table image of the aggregated statistics.

        @post
        - Loads config and sets up internal fields.
        - Ensures the @c results/ directory exists.
        - Builds the algorithm list and unique evaluation key list.
        - Counts per-algorithm result files and records the global maximum count.
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
        # after you finish populating self.iterations in __init__
        self.max_iterations = max(self.iterations.values()) if self.iterations else 0

    def createDir(self, path):
        """
        @brief Create a directory if it does not exist.

        @param path str
               Directory path to create.

        @return None
                Prints a message about creation, existence, or any error.
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
        @brief Check if all algorithms share the same iteration count.

        @details
        Returns True only if all values in the provided dict are identical. When
        True, @c self.totalFiles is set to that common value.

        @param iterations dict[str, int]
               Mapping from algorithm name to number of found result files.

        @return bool
                True if all counts match and the dict is non-empty; otherwise False.
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
        @brief Aggregate metrics per algorithm and write summary artifacts.

        @details
        For each algorithm directory under the run path, all JSON result files are
        loaded and their @c evaluation blocks are accumulated. The final per-algo
        metrics are computed as follows:
        - For keys deemed “success rate” (case/format variations accepted),
          the sum is divided by the global maximum iteration count observed across
          algorithms (@c self.max_iterations).
        - For all other metrics, the sum is divided by the number of files found
          for that algorithm.

        A pandas DataFrame is printed to stdout. If @c saveStatsImage was set in
        the constructor, a PNG table is saved under @c results/<runDetails>.png.
        A JSON summary is always written to
        @c results/<runDetails>Stats.json.

        @return None
                Side effects: prints a DataFrame, writes JSON (and optionally PNG).
        """
        def is_success_key(k: str) -> bool:
            # match SuccessRate / success_rate / success
            norm = k.replace("_", "").replace(" ", "").lower()
            return norm in {"successrate", "success"}

        meanAlgoData = {}
        for algo in self.algos:
            path = os.path.join(self.runPath, algo)
            sums = {k: 0.0 for k in self.evals}
            n = 0

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
                    for k in self.evals:
                        sums[k] += float(eval_block.get(k, 0.0))
                    n += 1

            if n == 0:
                print(f"No result files found for algo '{algo}' at {path}")
                meanAlgoData[algo] = {k: float("nan") for k in self.evals}
            else:
                out = {}
                for k in self.evals:
                    if is_success_key(k):
                        denom = self.max_iterations
                        out[k] = (sums[k] / denom) if denom > 0 else float("nan")
                    else:
                        out[k] = sums[k] / n
                meanAlgoData[algo] = out

        df = pd.DataFrame.from_dict(meanAlgoData, orient="index")[self.evals]
        print(df)

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

        out_json = os.path.join(self.resultsDir, f"{self.runDetails}Stats.json")
        with open(out_json, "w") as f:
            json.dump(meanAlgoData, f, indent=4)
