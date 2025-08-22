# 🛰️ SafePlan: A Benchmark for Safe and Optimal Path Planning

**SafePlan** is a benchmarking suite for evaluating classical and modern path planners in robotics with a strong emphasis on **safety, smoothness, and optimality**.  
Unlike existing frameworks (e.g., PathBench), which primarily measure path cost and runtime, SafePlan introduces **safety-critical and dynamics-aware metrics** that better reflect the requirements of real robotic systems.

---

## ✨ Key Features

- 🔹 **Planner-agnostic**: Works with any planner that outputs a path on a grid or occupancy map.  
- 🔹 **Safety-aware metrics**: Beyond length and time, includes clearance, risk exposure, danger zone violations.  
- 🔹 **Smoothness evaluation**: Turning angles, jerk (third derivative of position), zig-zag count.  
- 🔹 **Optimality & efficiency**: Path cost deviation from shortest path, success rate, computation time.  
- 🔹 **Extensible framework**: Add new metrics or planners with minimal boilerplate.  
- 🔹 **Batch evaluation**: Run thousands of randomized start-goal trials across multiple maps and export results to CSV/plots.

---

## 📊 Evaluation Metrics

SafePlan provides a **rich set of metrics** grouped into categories:

### 🔹 Efficiency
- **Path cost (m)** – total path length in meters  
- **Computation time (ms)** – wall-clock planning time  
- **Success rate (%)** – fraction of runs that reach the goal  
- **Distance to goal (m)** – residual distance if failure  
- **Nodes in path** – number of waypoints (before/after simplification)  

### 🔹 Smoothness
- **Turning angle (°)** – cumulative direction changes  
- **Jerk per meter** – Cartesian jerk (third finite difference) normalized by path length  
- **Zig-zag count** – number of rapid direction oscillations  

### 🔹 Safety
- **Minimum clearance (m)** – closest distance to any obstacle  
- **Clearance variability** – variance of obstacle clearance along the path  
- **Danger zone violations** – number of times path enters unsafe radii around obstacles  
- **Riskiness score** – sum of inverse distances to obstacles (higher = riskier path)  

### 🔹 Optimality
- **Optimal deviation (%)** – relative excess length compared to the true shortest path  
- **Optimality ratio** – path cost / optimal cost  

---

## 🛠️ Installation

```bash
git clone https://github.com/<your-username>/safeplan.git
cd safeplan
pip install -r requirements.txt
