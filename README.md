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

SafePlan provides a **rich set of metrics** :

# SafePlan Benchmarking Metrics (Summary)

- **Path Cost (m):**  
  Total geometric length of the path in meters, based on Euclidean distance between waypoints. Lower cost indicates a shorter and more efficient route.  

- **Planning Time (ms):**  
  The time taken by the planner to generate a path, measured in milliseconds. Faster planners achieve lower values.  

- **Success Rate (%):**  
  Percentage of planning attempts that successfully reach the goal. Higher success rate reflects reliability of the algorithm.  

- **Distance to Goal (m, failures):**  
  Distance from the last explored path node to the goal when planning fails. Smaller values show the planner got closer to success.  

- **Nodes in Path:**  
  Number of waypoints in the final path. Fewer nodes suggest a smoother, simpler, and more direct path.  

- **Peak Memory (KB):**  
  Maximum RAM used by the planner during execution. Lower values indicate more memory-efficient planning.  

- **Turning Angle (°):**  
  Sum of all angular deviations along the path. High turning angles mean sharper, less smooth turns.  

- **Jerk per Meter:**  
  Rate of change of acceleration normalized by distance. Captures path smoothness—lower jerk means more stable and comfortable trajectories.  

- **Optimal Deviation (%):**  
  Deviation of the computed path length from the known optimal path. Lower percentages mean closer-to-optimal planning.  

- **Minimum Clearance (m):**  
  The smallest distance between the path and any obstacle. Higher values reflect safer paths.  

- **Average Clearance (m):**  
  Mean clearance maintained along the path. Represents the overall safety margin to obstacles.  

- **Clearance Variability:**  
  Variance of clearance distances along the path. Low variability indicates consistent safety, while high variability indicates fluctuating risk.  

- **Danger Zone Violations:**  
  Number of times the path enters within a predefined danger radius around obstacles. Higher counts suggest higher collision risk.  
