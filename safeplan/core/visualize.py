"""
@file visualize.py
@brief Render 2D/3D occupancy grids and algorithm paths from run outputs.

@details
Provides convenient plotting for benchmark runs. For 2D grids, writes a PNG
with obstacles, start/goal markers, and optional path overlays. For 3D grids,
writes an interactive Plotly HTML (isosurface + point cloud + paths), and can
optionally save a Matplotlib 3D PNG snapshot when available.

Expected per-iteration input (for each algorithm):
- Located at `outputs/<runDetails>/<algo>/iter_<N>.json`
- Contains:
  - `envInfo.envName` : string name of the environment (e.g., "house_2d", "maze3d")
  - `startGoal.start` : start index tuple/list
  - `startGoal.goal`  : goal index tuple/list
  - `pathInfo.path`   : list of coordinates (tuples/lists)

Environment grid source (single, shared across algos for the same iter):
- Located at `configs/generate_grid/{envName}/{envName}_grid.json`
- Contains:
  - `grid` : N-D list/array (0=free, >0=occupied)

Artifacts:
- 2D PNG: `results/viz/{envName}_grid2d_iter<N>.png`
- 3D HTML: `results/viz/{envName}_grid3d_iter<N>.html`
- Optional 3D PNG (fallback snapshot): `results/viz/{envName}_grid3d_iter<N>.png`

@note
- Plotly is preferred for 3D (opens in a browser). If Plotly is unavailable,
  the code attempts a Matplotlib 3D fallback when possible.
- Obstacles are rendered as black (2D) or red (3D). Paths are colored and
  labeled by algorithm name.
- The function auto-creates `results/viz/` if it does not exist.
- Start/Goal are ALWAYS taken from the first algorithm's iter file.
- The GRID is ALWAYS taken from `configs/generate_grid/{envName}/{envName}_grid.json`, where
  `envName` is read from the first algorithm's iter file (`envInfo.envName`) and lowercased.

@see plotly.graph_objects, matplotlib.pyplot
"""

import os
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

# Try Matplotlib 3D
try:
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    HAS_MPL_3D = True
except Exception:
    HAS_MPL_3D = False

# Plotly (preferred for 3D)
try:
    import plotly.graph_objects as go
    import plotly.io as pio
    # Force a reliable renderer that opens in a browser
    pio.renderers.default = "browser"
    HAS_PLOTLY = True
except Exception:
    HAS_PLOTLY = False


def _ensure_outdir():
    """
    @brief Ensure the visualization output directory exists.

    @details
    Creates `results/viz/` if needed and returns the path.

    @return str
            Absolute/relative path of the visualization output directory.
    """
    out_dir = os.path.join("results", "viz")
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def _plot_3d_plotly_combo(occ_bool, paths, start, goal, title, html_path,
                          show_paths=True, max_points=120_000):
    """
    @brief Render a 3D scene with Plotly and write an HTML file.

    @details
    Draws occupied voxels using an isosurface plus a downsampled point cloud to
    guarantee visibility. Overlays optional algorithm paths and start/goal markers.
    Always writes an HTML file and prints its path.

    @param occ_bool numpy.ndarray
           Boolean occupancy mask (True = obstacle).
    @param paths dict[str, list[tuple]]
           Mapping from algorithm label to path (sequence of 3D points).
    @param start tuple[int, int, int]
           Start index in the 3D grid.
    @param goal tuple[int, int, int]
           Goal index in the 3D grid.
    @param title str
           Figure title.
    @param html_path str
           Output HTML file path.
    @param show_paths bool
           Whether to overlay paths on the plot.
    @param max_points int
           Maximum number of obstacle points to plot in the point-cloud overlay.
    """
    occ_u8 = occ_bool.astype(np.uint8)
    fig = go.Figure()

    # Isosurface of occupied voxels
    fig.add_trace(go.Isosurface(
        value=occ_u8,
        isomin=0.5, isomax=1.0, surface_count=1,
        opacity=0.55,
        colorscale=[[0, "red"], [1, "red"]],
        caps=dict(x_show=False, y_show=False, z_show=False),
        showscale=False,
        lighting=dict(ambient=0.5, diffuse=0.7, specular=0.2, roughness=0.8, fresnel=0.1),
        lightposition=dict(x=1000, y=1000, z=1000),
    ))

    # Downsampled point cloud overlay (guaranteed visible)
    coords = np.argwhere(occ_bool)  # (N,3)
    if len(coords) > 0:
        step = max(1, len(coords) // max_points)
        coords = coords[::step]
        fig.add_trace(go.Scatter3d(
            x=coords[:, 0], y=coords[:, 1], z=coords[:, 2],
            mode="markers", name="Obstacles",
            marker=dict(size=2, color="red", opacity=0.8)
        ))

    # Paths
    if show_paths and paths:
        for label, path in paths.items():
            if not path:
                continue
            a = np.asarray(path)
            fig.add_trace(go.Scatter3d(
                x=a[:, 0], y=a[:, 1], z=a[:, 2],
                mode="lines", name=label, line=dict(width=6)
            ))

    # Start / Goal
    fig.add_trace(go.Scatter3d(
        x=[start[0]], y=[start[1]], z=[start[2]],
        mode="markers", name="Start",
        marker=dict(size=7, color="yellow", line=dict(width=2, color="black"))
    ))
    fig.add_trace(go.Scatter3d(
        x=[goal[0]], y=[goal[1]], z=[goal[2]],
        mode="markers", name="Goal",
        marker=dict(size=7, color="red", line=dict(width=2, color="black"))
    ))

    fig.update_layout(
        title=title,
        scene=dict(xaxis_title="X", yaxis_title="Y", zaxis_title="Z", aspectmode="data"),
        margin=dict(l=0, r=0, b=0, t=40),
        legend=dict(itemsizing="constant")
    )

    fig.write_html(html_path, auto_open=True)
    print(f"[viz] Wrote 3D HTML to: {html_path}")


class Visualize:
    """
    @class Visualize
    @brief Visualize 2D and 3D environments and algorithm paths.

    @details
    - 2D mode: saves a PNG with full grid, optional path overlays, and start/goal.
    - 3D mode: writes a Plotly HTML (isosurface + point cloud + paths), optionally
      saves a Matplotlib 3D PNG snapshot.
    """

    def see(self, runConfigPath, iterNo, algos=None, graphTitle=None,
            show_paths=True, prefer_plotly=True, also_save_png_3d=False):
        """
        @brief Render a specific iteration across one or more algorithms.

        @details
        Loads run configuration, gathers envName from the first selected algorithm’s
        iter file for the given iteration, then loads the GRID from
        `configs/generate_grid/{envName}/{envName}_grid.json` (envName lowercased).
        Start/Goal and per-algorithm paths are loaded from the respective
        `iter_<N>.json` files. Output format depends on grid dimension:
        - 2D: writes `{envName}_grid2d_iter<N>.png`
        - 3D: writes `{envName}_grid3d_iter<N>.html` (Plotly) and, if requested and supported,
               also `{envName}_grid3d_iter<N>.png`

        @param runConfigPath str
               Path to the run configuration JSON.
        @param iterNo int
               Iteration number N (reads `iter_N.json`).
        @param algos list[str] or None
               Subset of algorithm names to visualize; defaults to all listed
               in the config when None.
        @param graphTitle str or None
               Plot title; uses a default if not provided.
        @param show_paths bool
               Whether to overlay algorithm paths.
        @param prefer_plotly bool
               Prefer Plotly for 3D visualization (recommended).
        @param also_save_png_3d bool
               Additionally save a Matplotlib 3D PNG snapshot when possible.

        @return None
                Writes files to `results/viz/` and prints artifact locations.
        """
        self.outputDir = "outputs"
        self.iterFile = f"iter_{iterNo}.json"
        self.graphTitle = graphTitle or "Path Planning Plots"
        self.show_paths = show_paths

        # Load run config
        with open(runConfigPath) as f:
            data = json.load(f)
        algos_all = [k["name"] for k in data["algoDetails"]]
        self.algos = algos or algos_all
        runPath = os.path.join(self.outputDir, data["runDetails"])

        # Load first algo iter file for envName/start/goal, and gather paths from all
        self.algoPaths, self.grid, self.start, self.goal = {}, None, None, None
        first_meta = None
        for algo in self.algos:
            p = os.path.join(runPath, algo, self.iterFile)
            with open(p) as f:
                j = json.load(f)
            self.algoPaths[algo] = j.get("pathInfo", {}).get("path", [])
            if first_meta is None:
                first_meta = j

        if first_meta is None:
            raise RuntimeError("[viz] Could not read any algo iter file(s).")

        # Start/Goal always from iter file
        self.start = tuple(first_meta["startGoal"]["start"])
        self.goal  = tuple(first_meta["startGoal"]["goal"])

        # Grid: read envName from iter file, then load from configs/generate_grid/env/env_grid.json
        env_raw = first_meta.get("envInfo", {}).get("envName")
        if not env_raw:
            raise KeyError("[viz] Missing envInfo.envName in iter file.")
        env_name = env_raw.lower()

        # Resolve repo root relative to runConfigPath: .../safeplan/safeplan/
        repo_root = Path(runConfigPath).resolve().parent.parent
        env_json = repo_root / "configs" / "generate_grid" / env_name / f"{env_name}_grid.json"
        if not env_json.is_file():
            raise FileNotFoundError(f"[viz] env grid file not found: {env_json}")

        with open(env_json) as f:
            env_data = json.load(f)
        self.grid = np.array(env_data["Grid"])

        dim = self.grid.ndim
        occ_mask = (self.grid > 0)
        occ_count = int(occ_mask.sum())
        print(f"[viz] env={env_name}, grid shape={self.grid.shape}, occupied={occ_count}/{occ_mask.size}")

        out_dir = _ensure_outdir()

        # 2D
        if dim == 2:
            fig, ax = plt.subplots(figsize=(12, 12))
            cmap = mcolors.ListedColormap(['white', 'black'])
            norm = mcolors.BoundaryNorm([-0.5, 0.5, 1.5], cmap.N)
            ax.imshow(self.grid, cmap=cmap, norm=norm, origin="upper")
            if self.show_paths:
                for label, path in self.algoPaths.items():
                    if not path:
                        continue
                    px, py = zip(*path)
                    ax.plot(py, px, linewidth=2, label=label)

            ax.scatter(self.start[1], self.start[0], c='yellow', s=100, edgecolors='black', label='Start')
            ax.scatter(self.goal[1],  self.goal[0],  c='red',    s=100, edgecolors='black', label='Goal')
            ax.set_title(self.graphTitle)
            if self.show_paths:
                ax.legend(loc="upper left")
            ax.set_aspect('equal')
            plt.tight_layout()

            out_png = os.path.join(out_dir, f"{env_name}_grid2d_iter{iterNo}.png")
            plt.savefig(out_png, dpi=160)
            plt.close(fig)
            print(f"[viz] Wrote 2D PNG to: {out_png}")
            return

        # 3D
        if dim == 3:
            if occ_count == 0:
                print("[viz] No occupied voxels detected. Check your 3D grid content.")
                return

            out_html = os.path.join(out_dir, f"{env_name}_grid3d_iter{iterNo}.html")

            if prefer_plotly and HAS_PLOTLY:
                _plot_3d_plotly_combo(
                    occ_bool=occ_mask,
                    paths=self.algoPaths,
                    start=self.start,
                    goal=self.goal,
                    title=self.graphTitle,
                    html_path=out_html,
                    show_paths=self.show_paths
                )

                # Optional: also save a quick PNG via Matplotlib 3D if available
                if also_save_png_3d and HAS_MPL_3D:
                    try:
                        fig = plt.figure(figsize=(9, 9))
                        ax = fig.add_subplot(111, projection='3d')
                        coords = np.argwhere(occ_mask)
                        step = max(1, len(coords) // 120_000)
                        coords = coords[::step]
                        ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2],
                                   s=2, c='red', alpha=0.1, depthshade=False)

                        ax.scatter(self.start[0], self.start[1], self.start[2],
                                   c='yellow', s=60, edgecolors='black')
                        ax.scatter(self.goal[0],  self.goal[1],  self.goal[2],
                                   c='red', s=60, edgecolors='black')
                        X, Y, Z = occ_mask.shape
                        ax.set_xlim(0, X); ax.set_ylim(0, Y); ax.set_zlim(0, Z)
                        try: ax.set_box_aspect((X, Y, Z))
                        except Exception: pass
                        ax.view_init(elev=25, azim=35)
                        ax.set_title(self.graphTitle)
                        out_png = os.path.join(out_dir, f"{env_name}_grid3d_iter{iterNo}.png")
                        plt.tight_layout(); plt.savefig(out_png, dpi=160); plt.close(fig)
                        print(f"[viz] Also wrote 3D PNG to: {out_png}")
                    except Exception as e:
                        print(f"[viz] Matplotlib PNG fallback failed: {e}")
                return

            # If Plotly missing, try Matplotlib
            if HAS_MPL_3D:
                fig = plt.figure(figsize=(9, 9))
                ax = fig.add_subplot(111, projection='3d')
                coords = np.argwhere(occ_mask)
                step = max(1, len(coords) // 150_000)
                coords = coords[::step]
                ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2],
                           s=1, c='red', depthshade=False)
                ax.scatter(self.start[0], self.start[1], self.start[2],
                           c='yellow', s=60, edgecolors='black')
                ax.scatter(self.goal[0],  self.goal[1],  self.goal[2],
                           c='red', s=60, edgecolors='black')
                X, Y, Z = occ_mask.shape
                ax.set_xlim(0, X); ax.set_ylim(0, Y); ax.set_zlim(0, Z)
                try: ax.set_box_aspect((X, Y, Z))
                except Exception: pass
                ax.view_init(elev=25, azim=35)
                ax.set_title(self.graphTitle)
                out_png = os.path.join(out_dir, f"{env_name}_grid3d_iter{iterNo}.png")
                plt.tight_layout(); plt.savefig(out_png, dpi=160); plt.close(fig)
                print(f"[viz] Wrote 3D PNG to: {out_png}")
                return

            print("[viz] Neither Plotly nor Matplotlib 3D available; cannot render 3D.")
            return

        # If we reach here, dimension > 3
        print("Can't plot above 3 dimensions")
