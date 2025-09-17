# SafePlan: A Benchmark for Safe and Optimal Path Planning

## Overview

**SafePlan** is a benchmarking suite for evaluating classical and modern path planning algorithms with a strong focus on **safety and optimality** in robotics.  
It compares traditional planners with safety-aware planners, enabling fair evaluation of how algorithms balance safety and efficiency.  

SafePlan also includes the proposed **OptiSafe** metric, designed to quantify the trade-off between path optimality and safety, providing a more realistic assessment of planner performance in safety-critical robotic applications.  


## Key Features

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
