# SafePlan: A Benchmark for Safe and Optimal Path Planning

## Overview

**SafePlan** is a benchmarking suite for evaluating classical and modern path planning algorithms with a strong focus on **safety and optimality** in robotics.  
It compares traditional planners with safety-aware planners, enabling fair evaluation of how algorithms balance safety and efficiency.  

SafePlan also includes the proposed **OptiSafe** metric, designed to quantify the trade-off between path optimality and safety, providing a more realistic assessment of planner performance in safety-critical robotic applications.  


## Key Features

- **Resume at any time**
  - If code crashes or stops due to any reason, it will resume from the last stopped point of iteration
  - Doesn't do repeated computation 

- **Batch experiments at scale**
  - Run thousands of randomized start–goal trials across multiple maps automatically.
  - Supports repeatable benchmarking with structured JSON outputs.

- **Reproducibility & logging**
  - Standardized JSON logs for each run with environment, path, and evaluation info.
  - Aggregated statistics in CSV/JSON and optional visualization outputs.

- **Visualization tools**
  - 2D plots of environments and paths.
  - 3D interactive visualization (via Plotly) for voxel maps and trajectories.

- **Efficient & stable execution**
  - Each run is sandboxed in subprocesses for memory isolation and reliability.
  - Lightweight and scalable to large benchmark runs.

- **Extensible framework**
  - Add new planners, environments, or metrics with minimal boilerplate.
  - Designed for all kinds of classical planners, sampling based and geometry based.

## Evaluation Metrics

SafePlan introduces a rich set of metrics that go **beyond path length and runtime**.  
They capture not only **efficiency**, but also **safety** and **smoothness** — key aspects for real robotic systems.

---

### 1. Path Cost (m)
- **Definition:** Total geometric length of the path from start to goal.  
- **Computation:** Sum of Euclidean distances between consecutive waypoints, scaled by cell size.   
- **Unit:** meters (m).  

### 2. Planning Time (ms)
- **Definition:** CPU time taken by the planner to compute a path.  
- **Computation:** Wall-clock difference between start and end of plan.  
- **Unit:** milliseconds (ms).  

### 3. Success Rate (%)
- **Definition:** Fraction of planning queries where a valid path is found.  
- **Computation:** (#successful plans ÷ #total attempts) × 100.  
- **Unit:** percentage (%).  

### 4. Distance to Goal (m, failures only)
- **Definition:** Remaining distance between the last explored path node and the goal when planning fails.  
- **Computation:** Euclidean distance between last path node and goal.  
- **Unit:** meters (m).  

### 5. Number of Nodes in Path
- **Definition:** Count of waypoints in the final path.  
- **Unit:** count (integer).  

### 6. Turning Angle (°)
- **Definition:** Sum of angular deviations between consecutive path segments.  
- **Computation:** Angles via dot product between vectors, summed across path.  
- **Unit:** degrees (°).  

### 7. Jerk per Meter
- **Definition:** Rate of change of acceleration along the path, normalized per unit distance.  
- **Computation:** Third derivative of position along discretized path.  
- **Unit:** dimensionless (jerk/m).  

### 8. Optimal Deviation (%)
- **Definition:** Difference between path cost and known optimal path cost.  
- **Computation:** ((path_cost − optimal_cost) ÷ optimal_cost) × 100.  
- **Unit:** percentage (%).  

### 9. Minimum Clearance (m)
- **Definition:** Smallest distance from path to nearest obstacle.  
- **Computation:** Distance transform (EDT) at sampled points.  
- **Unit:** meters (m).  

### 10. Average Minimum Clearance (m)
- **Definition:** Mean clearance across path points.  
- **Unit:** meters (m).  

### 11. Clearance Variability
- **Definition:** Variance of clearance values along the path.  
- **Unit:** meters².  

### 12. Danger Zone Violations
- **Definition:** Number of times path enters a predefined danger radius around obstacles.  
- **Computation:** Count of path points with clearance < danger radius.  
- **Unit:** count (integer).  

### 13. OptiSafe Index (Proposed)
- **Definition:** An index to measure balance between safety and optimality 
- **Unit:** float between 0 and 1.  


## Algorithms in SafePlan

SafePlan includes both **traditional planners** and **safety-aware planners**, allowing direct benchmarking on **safety vs. optimality** tradeoffs.

### 🔹 Traditional Planners
- **Dijkstra**  
  Classic graph search algorithm that guarantees shortest paths but can be computationally expensive on large grids.  
  Reference: (https://www.semanticscholar.org/paper/A-note-on-two-problems-in-connexion-with-graphs-Dijkstra/45786063578e814444b8247028970758bbbd0488)

- **A\***  
  Heuristic-guided search algorithm that combines optimality of Dijkstra with efficiency via heuristics.  
  Reference: (https://ieeexplore.ieee.org/document/4082128)

- **Voronoi Roadmap Planner**  
  Builds a roadmap from obstacle centroids using Voronoi diagrams, ensuring high-clearance paths.  
  Reference: (https://dl.acm.org/doi/10.1145/116873.116880)

- **RRT (Rapidly-Exploring Random Trees)**  
  Sampling-based motion planning method efficient for high-dimensional spaces.  
  Reference: (https://www.semanticscholar.org/paper/Rapidly-exploring-random-trees-%3A-a-new-tool-for-LaValle/d967d9550f831a8b3f5cb00f8835a4c866da60ad)

- **Weighted A\***  
  Variant of A\* that scales the heuristic to bias search toward speed vs. optimality tradeoffs.  
  Reference: (https://www.sciencedirect.com/science/article/pii/S000437020900068X)

---

### 🔹 Safety-Aware Planners
- **Unified Path Planner**  
  A safety-aware planner integrating safety heuristics  
  Reference: (https://arxiv.org/pdf/2505.23197)

- **SDF A\***  
  A\* variant augmented with Signed Distance Fields for safety margin enforcement.  
  Reference: (https://ieeexplore.ieee.org/document/10364842/)

- **Optimized A\***  
  An improved A\* with path smoothness and obstacle-clearance optimizations.  
  Reference: (https://www.mdpi.com/1424-8220/24/10/3149)

- **CBF-RRT (Control Barrier Function RRT)**  
  Enhances RRT with control barrier functions to guarantee safety constraints.  
  Reference: (https://arxiv.org/pdf/2011.06748)


## Installation

### Requirements
- **Python**: 3.9 or newer  
- **Core dependencies**:
  - `numpy`
  - `scipy`
  - `pandas`
  - `matplotlib`
  - `plotly`
  - `networkx`
  - `psutil`
  - `networkx`
  - `json`, `os`, `subprocess` (standard library)

All dependencies are listed in `requirements.txt`.

---

### Install from source

Clone the repository and install in editable mode:

```bash
git clone https://github.com/<your-username>/safeplan.git
cd safeplan
pip install -r requirements.txt
pip install -e .
```
## Usage

SafePlan experiments are configured via a JSON file (e.g., `run1.json`).  
Below is an example:

```json
{
  "runDetails": "run1",
  "evalDetails": [
    {"name": "PlanningTime"},
    {"name": "MinimumClearance","args": {"pointSamples":100}},
    {"name": "AverageMinimumClearance","args": {"pointSamples":100}},
    {"name": "ClearanceVariability","args": {"pointSamples":100}},
    {"name": "DangerViolations","args": {"pointSamples":100,"dangerRadius":0.2}},
    {"name": "TurningAngle"},
    {"name": "JerkPerMeter"},
    {"name": "SuccessRate"},
    {"name": "OptimalDeviation"},
    {"name": "PathCost"},
    {"name": "DistanceToGoal"},
    {"name": "OptiSafeIndex","args": {"pointSamples":100,"knn":1000}},
    {"name": "NodesInPath", "args": {"type":"RDP","epsilon":0.01}}
  ],
  "algoDetails": [
    {"name": "VoronoiPlanner","args": {"pointSamples":100,"knn":1000}},
    {"name": "UPP","args": {"alpha":0.5,"beta":2,"radius":5,"epsilon":0.01}},
    {"name": "RRT","args": {"maxIter":10000,"goalSampleRate":0.05,"stepSize":7,"pointSamples":100}},
    {"name": "CBFRRT","args": {"maxIter":10000,"goalSampleRate":0.05,"stepSize":7,"pointSamples":100,"gamma1":2,"gamma2":2}},
    {"name": "AStar"},
    {"name": "Dijkstra"},
    {"name": "WeightedAStar","args": {"weight":1.2}},
    {"name": "SDFAStar" , "args": {"k1":5,"k2":5}},
    {"name": "OptimizedAStar" , "args": {"turnPenaltyCoefficients":0.5,"safetyDistGridRadius":100,"maxInflateIter":100,"pointSamples":100}}
  ],
  "envDetails": [
    {
      "generateGrid": [
        {"name": "env12"},
        {"name": "env2"}
      ]
    }
  ]
}
```

##  Programmatic API

You can run SafePlan directly from Python by importing the main API and passing a path to your run config JSON.

```python
from safeplan.main import SafePlan
from safeplan.core.stats import Stats
from safeplan.core.visualize import Visualize

# 1) Run the benchmark
sf = SafePlan("/home/run1.json")
sf.benchmark()

# 2) Compute aggregate statistics across all outputs
st = Stats("/home/run1.json")
st.compute()

# 3) Visualize one iteration (2D → PNG, 3D → Plotly HTML)
viz = Visualize()
viz.see("/home/run1.json", iterNo=1, prefer_plotly=True)
```

SafePlan experimental results  in results folder on calling stats for planners for various metrices are shown in json form as follows:
```json
{
    "VoronoiPlanner": {
        "PlanningTime": 34.41541785260046,
        "MinimumClearance": 1.1250388913720941,
        "AverageMinimumClearance": 2.352815958430445,
        "ClearanceVariability": 28.298456912893112,
        "DangerViolations": 25.48421052631579,
        "TurningAngle": 50.832074296827976,
        "JerkPerMeter": 0.1293378835798869,
        "SuccessRate": 0.78,
        "OptimalDeviation": -0.04541845530415791,
        "PathCost": 14.881445458621005,
        "DistanceToGoal": 3.3916573898608524,
        "OptiSafeIndex": 0.7065313469330626,
        "NodesInPath": 2.294736842105263
    },
    "UPP": {
        "PlanningTime": 596.5560107800275,
        "MinimumClearance": 1.1739753421354147,
        "AverageMinimumClearance": 2.3505294974739472,
        "ClearanceVariability": 32.347431493838286,
        "DangerViolations": 1258.0,
        "TurningAngle": 391.1796618233935,
        "JerkPerMeter": 0.11140010671457004,
        "SuccessRate": 1.0,
        "OptimalDeviation": 0.04545741951548409,
        "PathCost": 17.57754551728892,
        "DistanceToGoal": 0.0,
        "OptiSafeIndex": 0.7248084252595861,
        "NodesInPath": 10.58
    },
    "RRT": {
        "PlanningTime": 3710.6396523877615,
        "MinimumClearance": 0.7774365236197455,
        "AverageMinimumClearance": 2.1454706756834785,
        "ClearanceVariability": 32.811645103595026,
        "DangerViolations": 1158.6530612244899,
        "TurningAngle": 1854.150451654323,
        "JerkPerMeter": 0.8274345209351016,
        "SuccessRate": 0.94,
        "OptimalDeviation": 0.3328127469478287,
        "PathCost": 21.397049521927464,
        "DistanceToGoal": 0.8620635171868056,
        "OptiSafeIndex": 0.405458422344748,
        "NodesInPath": 53.19387755102041
    },
    "CBFRRT": {
        "PlanningTime": 2878.820585159561,
        "MinimumClearance": 0.7667586101167566,
        "AverageMinimumClearance": 2.2535040753205755,
        "ClearanceVariability": 33.721935597252006,
        "DangerViolations": 879.6595744680851,
        "TurningAngle": 1909.2118279618126,
        "JerkPerMeter": 0.8389019157175471,
        "SuccessRate": 0.91,
        "OptimalDeviation": 0.3399416476233767,
        "PathCost": 22.32407924888354,
        "DistanceToGoal": 0.7363309040081364,
        "OptiSafeIndex": 0.39376363852758384,
        "NodesInPath": 55.48936170212766
    },
    "AStar": {
        "PlanningTime": 174.72529752001265,
        "MinimumClearance": 1.0151776710837503,
        "AverageMinimumClearance": 2.2424191120707406,
        "ClearanceVariability": 34.30131815730188,
        "DangerViolations": 15076.0,
        "TurningAngle": 197.23881127868557,
        "JerkPerMeter": 0.056250231194724935,
        "SuccessRate": 1.0,
        "OptimalDeviation": 0.0,
        "PathCost": 17.440011462679397,
        "DistanceToGoal": 0.0,
        "OptiSafeIndex": 0.5991712447987082,
        "NodesInPath": 6.35
    },
    "Dijkstra": {
        "PlanningTime": 6141.7088189300275,
        "MinimumClearance": 1.0288958202911573,
        "AverageMinimumClearance": 2.2705344720789187,
        "ClearanceVariability": 34.22195930119145,
        "DangerViolations": 13818.0,
        "TurningAngle": 243.10344947451517,
        "JerkPerMeter": 0.06028571145884985,
        "SuccessRate": 1.0,
        "OptimalDeviation": -0.0052263642391577984,
        "PathCost": 17.214877082475255,
        "DistanceToGoal": 0.0,
        "OptiSafeIndex": 0.5965422912166155,
        "NodesInPath": 7.36
    },
    "WeightedAStar": {
        "PlanningTime": 153.34242371997675,
        "MinimumClearance": 1.0151776710837503,
        "AverageMinimumClearance": 2.241324597144535,
        "ClearanceVariability": 34.311112829724934,
        "DangerViolations": 15098.0,
        "TurningAngle": 392.97292911634526,
        "JerkPerMeter": 0.08123820680601991,
        "SuccessRate": 1.0,
        "OptimalDeviation": 0.004609068328356913,
        "PathCost": 17.514478020814526,
        "DistanceToGoal": 0.0,
        "OptiSafeIndex": 0.5999460252793478,
        "NodesInPath": 10.66
    },
    "SDFAStar": {
        "PlanningTime": 2218.002218519984,
        "MinimumClearance": 1.68614096294055,
        "AverageMinimumClearance": 2.6620601013107192,
        "ClearanceVariability": 19.980510612103856,
        "DangerViolations": 2476.0,
        "TurningAngle": 1252.5719528909428,
        "JerkPerMeter": 0.21314981311808062,
        "SuccessRate": 1.0,
        "OptimalDeviation": 0.09742635893468785,
        "PathCost": 19.207122069010566,
        "DistanceToGoal": 0.0,
        "OptiSafeIndex": 0.722120828235946,
        "NodesInPath": 22.31
    },
    "OptimizedAStar": {
        "PlanningTime": 483.78120382995337,
        "MinimumClearance": 1.0693436713052826,
        "AverageMinimumClearance": 2.6235459319424956,
        "ClearanceVariability": 32.59846635393538,
        "DangerViolations": 56.84,
        "TurningAngle": 89.06605471889655,
        "JerkPerMeter": 0.2828841739505983,
        "SuccessRate": 1.0,
        "OptimalDeviation": 0.10790189307066612,
        "PathCost": 19.070090151242606,
        "DistanceToGoal": 0.0,
        "OptiSafeIndex": 0.6469495215941641,
        "NodesInPath": 3.09
    }
}

```

## Environments

SafePlan currently provides **12 benchmark environments**:

- **2D Large (1000×1000):** 5 maps  
- **2D Medium (100×100):** 5 maps  
- **3D Volumetric (100×100×100):** 2 maps  

These cover a diverse range of planning challenges:
- Large-scale grids for stress testing scalability  
- Medium-sized grids for rapid prototyping  
- Full 3D voxel maps for UAV and mobile robot planning  

##  Extending SafePlan

SafePlan is designed to be **modular and extensible**, so you can easily add your own planners, evaluation metrics, or environments.  
This makes it straightforward for researchers and developers to benchmark **new ideas** while reusing the existing infrastructure.



###  Adding a New Planner
To add a new planner:
1. Create a new file inside `safeplan/algos/` (e.g., `my_planner.py`).
2. Inherit from `BasePlanner` and implement the `plan(start, goal, grid)` method.
3. Register it in your JSON config under `"algoDetails"`.

**Example:**

`safeplan/algos/my_planner.py`
```python
from .baseplanner import BasePlanner

class MyPlanner(BasePlanner):
    def plan(self, start, goal, grid):
        # Dummy straight-line planner
        path = [start, goal]
        success = 1
        info = ["Trivial straight path"]
        return success, path, info
```
You can update planner details as in adding in json form:
```json
"algoDetails": [
    {"name": "MyPlanner"}
]
```

###  Adding a New Metric
To add a new metric:
1. Create a new file inside `safeplan/evals/` (e.g., `my_metric.py`).
2. Inherit from `BaseEval` and implement the `eval(start,goal,grid,cellSize,path) ` method.
3. Register it in your JSON config under `"evalDetails"`.

**Example:**

`safeplan/evals/my_metric.py`
```python
from .baseeval import BaseEval

class MyMetric(BaseEval):
    def eval(self, start, goal, grid, cellSize, path):
        # Example: count path length in steps
        if not path:
            return float("inf")
        return len(path)

```
You can update eval details as in adding in json form:
```json
"evalDetails": [
    {"name": "MyMetric"}
]
```

### Adding a New Environment

SafePlan lets you plug in new environment generators that return occupancy grids and start/goal pairs in a consistent format. You can either **use the built-in `GenerateGrid`** (loads polygons from JSON, rasterizes to a grid, caches a `_grid.json`) or **create your own environment** by subclassing `BaseEnv`.

---

Option A — Use the built-in `GenerateGrid`

`GenerateGrid` reads a scene JSON from:
safeplan/configs/generate_grid/<YOUR_ENV>/<YOUR_ENV>.json
and writes/loads a cache:
safeplan/configs/generate_grid/<YOUR_ENV>/<YOUR_ENV>_grid.json


What it does:
- Loads grid size, dimension, cell size, and polygon obstacles.
- Rasterizes polygons to an **N-D occupancy grid** (`0=free`, `1=obstacle`) via half-space checks.
- Marks polygon vertices as occupied.
- Optionally samples **random start/goal pairs** in free cells.
- **Caches** the generated grid and pairs to `<name>_grid.json`.  
  If the cache exists and is readable, it is used directly (fast path).

Scene JSON schema (example):
```json
{
  "gridSize": 100,
  "dimensionOfGrid": 2,
  "cellSize": 0.1,
  "envName": "SimpleRoom",
  "envDes": "A 2D room with convex polygon obstacles",
  "randomStartGoal": true,
  "numStartGoals": 50,
  "startGoalPairs": [],
  "polygons": [
    { "polygon": [[20,20],[40,20],[40,40],[20,40]] },
    { "polygon": [[60,60],[80,60],[80,80],[60,80]] }
  ]
}
```

Add to your run config:
```json
"envDetails": [
  { "generateGrid": [ { "name": "SimpleRoom" }, { "name": "WarehouseA" } ] }
]

```

Option B — Create a Custom Environment (subclass BaseEnv)

1. Create a file, e.g. safeplan/envs/my_env.py, that inherits BaseEnv and implements getmap():

```python
# safeplan/envs/my_env.py
from .baseenv import BaseEnv
import numpy as np

class MyEnv(BaseEnv):
    def __init__(self, size=64, cell_size=0.1):
        super().__init__()
        self.size = int(size)
        self.cell_size = float(cell_size)

    def getmap(self):
        # Build a simple empty 2D grid with one obstacle line
        grid = np.zeros((self.size, self.size), dtype=np.uint8)
        grid[self.size//2, :] = 1  # a horizontal wall

        envName = "MyEnv"
        envDes  = "Custom demo environment with a single wall"
        startGoalPairs = [
            {"start": (5, 5), "goal": (self.size-6, self.size-6)}
        ]
        return grid, self.cell_size, envName, envDes, startGoalPairs
```

2. Register it in your run config by referencing it in code.
Two patterns are common:

Config-driven (extend SafePlan.setUpEnvs() to load from JSON).

Direct instantiation if you’re experimenting.

Config-driven JSON example:

```json
"envDetails": [
  { "generateGrid": [ { "name": "SimpleRoom" } ] },
  { "customEnvs": [ { "class": "MyEnv", "args": { "size": 128, "cell_size": 0.05 } } ] }
]
```
Expected Return Contract (getmap())

Any environment (including GenerateGrid) must return:

grid : numpy.ndarray with shape (N1, N2[, N3, ...]), values {0=free, 1=obstacle}

cellSize : float (meters per cell, or chosen unit)

envName : str

envDes : str

startGoalPairs : list of objects like:

## Caching & Reloading (GenerateGrid)

First run: GenerateGrid loads <name>.json, rasterizes polygons, optionally samples random pairs, and writes <name>_grid.json.

Subsequent runs: If <name>_grid.json is present and readable, it skips generation and loads from cache.

Tip: Delete the _grid.json cache to force regeneration after changes to the base scene JSON.

## Random Start/Goal Pairs (GenerateGrid)

If "randomStartGoal": true, SafePlan will sample numStartGoals distinct pairs in free cells.

If "randomStartGoal": false, SafePlan uses the provided "startGoalPairs" array as-is.

# Troubleshooting

“File exists and is readable” but scene didn’t change: Delete the cached _grid.json to regenerate.

No obstacles appear: Check your polygons are convex and have valid vertex order; ConvexHull requires non-degenerate input.

Invalid pairs: Ensure start/goal lie within grid bounds and on free cells (grid[...] == 0).

3D grids: Use dimensionOfGrid: 3 and provide 3D polygons (polyhedra faces) that form valid convex hulls.

## Cite SafePlan

If you use **SafePlan** in your research, please cite:

```bibtex
@misc{,
  doi = {},
  url = {},
  author = {},
  keywords = {},
  title = {},
  publisher = {},
  year = {},
  copyright = {}
}
```
