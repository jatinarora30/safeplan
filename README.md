# Measuring Balance: The OptiSafe Index and SafePlan Benchmark for Safe Navigation

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
  - Aggregated statistics in JSON and optional visualization outputs.

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


- **A\***  
  Heuristic-guided search algorithm that combines optimality of Dijkstra with efficiency via heuristics.  
  Reference: (https://ieeexplore.ieee.org/document/4082128)

- **Voronoi Roadmap Planner**  
  Builds a roadmap from obstacle centroids using Voronoi diagrams, ensuring high-clearance paths.  
  Reference: (https://dl.acm.org/doi/10.1145/116873.116880)

- **RRT (Rapidly-Exploring Random Trees)**  
  Sampling-based motion planning method efficient for high-dimensional spaces.  
  Reference: (https://www.semanticscholar.org/paper/Rapidly-exploring-random-trees-%3A-a-new-tool-for-LaValle/d967d9550f831a8b3f5cb00f8835a4c866da60ad)


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

- **FS Planner**  
  Uses an integrated inverse distance, on lazy theta AStar and also gives a limit on maximum neighbours explored in goal and safe direction
  Reference: (https://arxiv.org/pdf/2505.24024)

- **Safe A\***  
  Safe A*: This approach is conceptually inspired by the inflation layer in the ROS Navigation framework wher traversal costs are increased in proportion to proximity to obstacles. Unlike Nav2, which uses an exponential decay function to inflate costs, we adopt a simplified inverse-distance formulation to achieve a similar safety bias.


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
  "runDetails": "run2",
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
    {"name": "VoronoiPlanner","args": {"pointSamples":100,"knn":100}},
    {"name": "AStar"},
    {"name": "RRT","args": {"maxIter":10000,"goalSampleRate":0.05,"stepSize":7,"pointSamples":100}},

    {"name": "UPP","args": {"alpha":0.5,"beta":2,"radius":5,"epsilon":0.01}},
    {"name": "CBFRRT","args": {"maxIter":10000,"goalSampleRate":0.05,"stepSize":7,"pointSamples":100,"gamma1":2,"gamma2":2}},
    {"name": "FSPlanner" ,"args": {"pointSamples":100,"cw":1, "epsilon":0.01,"maxNeigh":5}},
    {"name": "SDFAStar" , "args": {"k1":5,"k2":5}},
    {"name": "OptimizedAStar" , "args": {"turnPenaltyCoefficients":0.5,"safetyDistGridRadius":100,"maxInflateIter":100,"pointSamples":100}},
    {"name": "SafeAStar" ,"args": {"safetyFactor":4,"epsilon":0.01}}
  ],
  "envDetails": [
    {
      "generateGrid": [
       
        {"name": "env1" }
      ]    }
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

SafePlan simulation results  for 12000 iteration 1000 random start and goal for each planner.
| Algorithm        | Planning Time | Min Clearance | Avg Min Clearance | Clearance Var | Danger Violations | Turning Angle | Jerk / m | Success Rate | Opt Dev  | Path Cost | Dist to Goal | OptiSafe Index | Nodes in Path |
|------------------|---------------|----------------|--------------------|----------------|--------------------|----------------|------------|---------------|-----------|------------|----------------|----------------|----------------|
| AStar            | 165.05        | 0.49           | 1.49               | 32.25          | 11269.93           | 213.39         | 0.080     | 1.000         | 0.0000    | 14.36      | 0.00           | 0.642          | 6.75           |
| CBFRRT           | 1919.27       | 0.39           | 1.43               | 28.24          | 218.33             | 1515.21        | 0.911     | 0.749         | 0.3139    | 16.72      | 0.57           | 0.454          | 42.89          |
| FSPlanner        | 1183.95       | 0.58           | 1.71               | 29.69          | 369.68             | 256.30         | 0.091     | 1.0       | -0.0107   | 14.20      | 0.00           | 0.754          | 7.63           |
| OptimizedAStar   | 414.01        | 0.55           | 1.81               | 31.76          | 24.27              | 110.75         | 0.411     | 1.0        | 0.1131    | 15.79      | 0.00           | 0.608          | 3.37           |
| RRT              | 1765.69       | 0.39           | 1.44               | 28.39          | 217.88             | 1512.48        | 0.916     | 0.757         | 0.3154    | 16.70      | 0.56           | 0.454          | 42.80          |
| SDFAStar         | 2103.93       | 0.95           | 1.78               | 21.15          | 1006.36            | 1372.81        | 0.290     | 1.0        | 0.1034    | 16.08      | 0.00           | 0.775          | 24.73          |
| SafeAStar        | 479.59        | 0.58           | 1.69               | 28.82          | 184.99             | 824.06         | 0.197     | 1.0        | 0.0344    | 14.63      | 0.00           | 0.794          | 20.34          |
| UPP              | 1157.21       | 0.58           | 1.53               | 31.24          | 352.43             | 401.36         | 0.147     | 1.000         | 0.0369    | 14.49      | 0.00           | 0.773          | 10.90          |
| VoronoiPlanner   | 30.01         | 0.62           | 1.61               | 25.84          | 4.83               | 37.63          | 0.104     | 0.614         | -0.0707   | 13.49      | 2.59           | 0.689          | 2.08           |


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
