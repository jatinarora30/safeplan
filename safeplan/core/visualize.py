import json
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

class Visualize:
    """
    @class Visualize
    @brief Utility to visualize 2D and 3D path planning results.

    This class loads a run configuration JSON file and corresponding
    algorithm iteration files to plot planned paths in either
    grid-based (2D) or voxel-based (3D) environments.

    The plots display:
      - Obstacles (black)
      - Free space (white)
      - Algorithm-generated paths (colored lines)
      - Start and goal positions (highlighted markers)
    """

    def see(self, runConfigPath, iterNo, algos=None, graphTitle=None):
        """
        @brief Render 2D or 3D paths for one or more algorithms.

        This method reads the run configuration JSON file, retrieves
        algorithm iteration results for a given iteration number, and
        plots the environment grid/voxels with paths overlaid.

        @param runConfigPath
            Path to the run configuration JSON file. This file must
            contain keys: `algoDetails`, `evalDetails`, and `runDetails`.
        @param iterNo
            Iteration number to load (`iter_<iterNo>.json`).
        @param algos
            Optional list of algorithm names to visualize. If None,
            all algorithms listed in `algoDetails` are used.
        @param graphTitle
            Optional title for the generated plot. Defaults to
            `"Path Planning Plots"`.

        @return
            None. A matplotlib window is shown with the visualization.

        @details
            - Searches for algorithm outputs under:
              `outputs/<runDetails>/<algo>/iter_<iterNo>.json`
            - Each iteration file must contain:
                - `pathInfo.path` : list of coordinates
                - `startGoal.start` / `startGoal.goal`
                - `envInfo.grid` : 2D or 3D array

        @exception FileNotFoundError
            If the run configuration file or any iteration file cannot
            be found on disk.
        @exception KeyError
            If required keys are missing in the JSON structures.
        @exception ValueError
            If the environment dimensionality is greater than 3.

        @note
            - 2D environments are plotted using `imshow` with obstacles
              in black and paths as lines.
            - 3D environments are plotted using voxel rendering and
              3D line plots.
            - Dimensions above 3 cannot be visualized.

        @example
            @code{.py}
            vis = Visualize()
            vis.see("configs/run_001.json", iterNo=5,
                    algos=["A*", "UPP"],
                    graphTitle="Comparison of Paths")
            @endcode
        """
        self.algos=algos
        self.outputDir = "outputs"
        self.iterFile="iter_"+str(iterNo)+".json"
        self.graphTitle=graphTitle
        if self.graphTitle==None:
            self.graphTitle="Path Planning Plots"
            
        
        self.runConfigPath = runConfigPath
        with open(self.runConfigPath) as file:
            data = json.load(file)
        self.algosDetails = data["algoDetails"]
        self.evalsDetails = data["evalDetails"]
        self.runDetails = data["runDetails"]
        print(self.algosDetails)
        
        if self.algos==None:
            self.algos=[]
        
            for k in self.algosDetails:
                self.algos.append(k["name"])
  
        self.runPath = os.path.join(self.outputDir, self.runDetails)
                
        print(self.algos,self.runPath)
        self.algoPaths={}
        
        for algo in self.algos:
            path = os.path.join(self.runPath, algo,self.iterFile)
            with open(path) as file:
                data = json.load(file)
                self.algoPaths[algo]=data["pathInfo"]["path"]
                self.start=data["startGoal"]["start"]
                self.goal=data["startGoal"]["goal"]
                self.grid=data["envInfo"]["grid"]
                
            self.grid=np.array(self.grid)
        
        self.dimension=len(self.grid.shape)
        
        if self.dimension==2:
            
            # Plot Paths
            plt.figure(figsize=(12, 12))
            cmap = mcolors.ListedColormap(['white', 'black'])
            bounds = [-0.5, 0.5, 1.5]
            norm = mcolors.BoundaryNorm(bounds, cmap.N)
            plt.imshow(self.grid, cmap=cmap, norm=norm, origin="upper")

            for label, path in self.algoPaths.items():
                if path:
                    path_x, path_y = zip(*path)
                    plt.plot(path_y, path_x, linewidth=2, label=label)

            plt.scatter(self.start[1], self.start[0], color='yellow', s=100, edgecolor='black', label='Start')
            plt.scatter(self.goal[1], self.goal[0], color='red', s=100, edgecolor='black', label='Goal')
            plt.title(self.graphTitle)
            plt.legend(loc="upper left")
            plt.grid(True, color="gray", linestyle='--', linewidth=0.5)
            plt.show()
                    
        elif self.dimension==3:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.voxels(self.grid, facecolors='red', edgecolor='k', alpha=0.5)
            for label, path in self.algoPaths.items():
                if path:
                    path_x, path_y,path_z = zip(*path)
                    ax.plot(path_x, path_y,path_z, linewidth=2, label=label)
            ax.scatter(self.start[0], self.start[1],self.start[2], color='yellow', s=100, edgecolor='black', label='Start')
            ax.scatter(self.goal[0], self.goal[1],self.goal[2], color='red', s=100, edgecolor='black', label='Goal')

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(self.graphTitle)
            plt.legend()
            plt.show()

            
        else:
            print("Can't plot above 3 dimesions")
