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
import numpy as np

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
        loaded and their @c evaluation blocks are accumulated.

        - For keys deemed “success rate” (case/format variations accepted),
          we treat each iteration as a 0/1 success indicator, pad missing
          iterations with zeros up to @c self.max_iterations, and compute:
              mean = total_successes / max_iterations
              std  = std dev over max_iterations 0/1 samples.
        - For all other metrics, mean and std are computed over the per-file
          values available for that algorithm.

        Two DataFrames are printed: means and standard deviations.
        A JSON summary is written to results/<runDetails>Stats.json with
        structure:
            { algo: { metric: { "mean": ..., "std": ... }, ... }, ... }

        If @c saveStatsImage is True, a PNG table of "mean±std" is also saved.
        """
        def is_success_key(k: str) -> bool:
            # match SuccessRate / success_rate / success
            norm = k.replace("_", "").replace(" ", "").lower()
            return norm in {"successrate", "success"}

        meanAlgoData = {}
        stdAlgoData = {}

        for algo in self.algos:
            path = os.path.join(self.runPath, algo)
            # store all per-file values for each metric
            values = {k: [] for k in self.evals}

            for _, _, files in os.walk(path):
                for fname in files:
                    if not fname.lower().endswith(".json"):
                        continue
                    fpath = os.path.join(path, fname)
                    try:
                        with open(fpath, "r") as f:
                            data = json.load(f)
                        eval_block = data.get("evaluation", {})
                    except Exception as e:
                        print(f"Skipping '{fpath}': {e}")
                        continue

                    for k in self.evals:
                        values[k].append(float(eval_block.get(k, 0.0)))

            # If no files found for this algo
            any_vals = any(len(v) > 0 for v in values.values())
            if not any_vals:
                print(f"No result files found for algo '{algo}' at {path}")
                meanAlgoData[algo] = {k: float("nan") for k in self.evals}
                stdAlgoData[algo] = {k: float("nan") for k in self.evals}
                continue

            algo_means = {}
            algo_stds = {}

            for k in self.evals:
                arr_raw = np.array(values[k], dtype=float)

                if arr_raw.size == 0:
                    algo_means[k] = float("nan")
                    algo_stds[k] = float("nan")
                    continue

                if is_success_key(k):
                    # Treat each value as success count/indicator per iteration.
                    # Pad with zeros up to max_iterations so that:
                    #   mean = total_successes / max_iterations (as before),
                    #   std  = std dev across all iterations (0/1 style).
                    if self.max_iterations > 0:
                        num_iter = arr_raw.size
                        if self.max_iterations > num_iter:
                            pad = np.zeros(self.max_iterations - num_iter, dtype=float)
                            arr_all = np.concatenate([arr_raw, pad])
                        else:
                            arr_all = arr_raw  # already at max_iterations

                        mean_k = arr_all.mean()            # sum / max_iterations
                        std_k = arr_all.std(ddof=0)        # population std
                    else:
                        mean_k = float("nan")
                        std_k = float("nan")
                else:
                    # Regular metric: mean/std over available runs
                    mean_k = arr_raw.mean()
                    std_k = arr_raw.std(ddof=0)

                algo_means[k] = float(mean_k)
                algo_stds[k] = float(std_k)

            meanAlgoData[algo] = algo_means
            stdAlgoData[algo] = algo_stds

        # DataFrames for convenience
        df_mean = pd.DataFrame.from_dict(meanAlgoData, orient="index")[self.evals]
        df_std = pd.DataFrame.from_dict(stdAlgoData, orient="index")[self.evals]

        print("=== Mean metrics ===")
        print(df_mean)
        print("\n=== Std dev over iterations (per algo, per metric) ===")
        print(df_std)

        # Optional image: show "mean±std" as in papers
        if self.saveStatsImage:
            df_disp = df_mean.copy().astype(str)
            for algo in self.algos:
                for k in self.evals:
                    m = df_mean.loc[algo, k]
                    s = df_std.loc[algo, k]
                    if pd.isna(m) or pd.isna(s):
                        df_disp.loc[algo, k] = "nan"
                    else:
                        df_disp.loc[algo, k] = f"{m:.3f}±{s:.3f}"

            fig = plt.figure(
                figsize=(
                    max(6, len(self.evals) * 1.2),
                    max(3, len(self.algos) * 0.6),
                )
            )
            ax = plt.subplot(111, frame_on=False)
            ax.xaxis.set_visible(False)
            ax.yaxis.set_visible(False)
            table(ax, df_disp, loc="center")
            plt.tight_layout()
            out_png = os.path.join(self.resultsDir, f"{self.runDetails}.png")
            plt.savefig(out_png, dpi=200)
            plt.close(fig)

        # JSON summary: algo -> metric -> {mean, std}
        summary = {}
        for algo in self.algos:
            summary[algo] = {}
            for k in self.evals:
                summary[algo][k] = {
                    "mean": meanAlgoData[algo].get(k, float("nan")),
                    "std": stdAlgoData[algo].get(k, float("nan")),
                }

        out_json = os.path.join(self.resultsDir, f"{self.runDetails}Stats.json")
        with open(out_json, "w") as f:
            json.dump(summary, f, indent=4)
