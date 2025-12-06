import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import fftconvolve
from scipy.ndimage import distance_transform_edt


class SafetyFieldDemo:
    def __init__(self, grid, R=5, epsilon=1e-3):
        """
        grid: 2D numpy array
              0 = free, 1 = obstacle
        R:    safety kernel Chebyshev radius
        """
        self.grid = grid.astype(np.float32)
        self.R = R
        self.epsilon = epsilon
        self.preSafety = None
        self.D = None

    def precomputeSafety(self):
        grid = self.grid
        obs = (grid == 1).astype(np.float32)
        R = int(self.R)

        # If no obstacles or invalid radius → no safety
        if R <= 0 or not np.any(obs):
            zero = np.zeros_like(obs, dtype=np.float32)
            self.preSafety = zero
            self.D = distance_transform_edt(grid == 0).astype(np.float32)
            return

        # --- Build Chebyshev radius stencil ---
        nd = obs.ndim  # should be 2
        grids = np.ogrid[tuple(slice(-R, R + 1) for _ in range(nd))]
        from functools import reduce
        d_inf = reduce(np.maximum, (np.abs(g) for g in grids)).astype(np.float32)

        # Kernel for method A: top-hat over Chebyshev ball
        K_count = np.zeros_like(d_inf, dtype=np.float32)
        K_count[d_inf <= R] = 1.0

        # Kernel for method B: inverse-distance over Chebyshev ball
        K_inv = np.zeros_like(d_inf, dtype=np.float32)
        # d_inf == 0 is the centre (self), we skip it; start at distance 1
        mask_inv = (d_inf > 0) & (d_inf <= R)
        K_inv[mask_inv] = 1.0 / (d_inf[mask_inv] + self.epsilon)

        # Convolve obstacles with inverse-distance kernel
        S_sumInv = fftconvolve(obs, K_inv, mode="same").astype(np.float32)

        # Distance transform (kept exactly as in your original code)
        self.D = distance_transform_edt(grid == 0).astype(np.float32)

        free = (grid == 0)
        D = self.D  # not strictly needed here but kept for completeness

        preB = np.zeros_like(D, dtype=np.float32)
        maskB = free & (S_sumInv > 0)

        # here S_sumInv already *is* the sum of 1/(distance + eps) over obstacles
        preB[maskB] = S_sumInv[maskB]
        self.preSafety = preB.astype(np.float32)


def build_test_grid():
    """
    Create a simple test grid with some obstacles.
    0 = free, 1 = obstacle
    """
    H, W = 50, 50
    grid = np.zeros((H, W), dtype=np.int32)

    # Add a vertical wall
    grid[10:40, 20] = 1

    # Add a horizontal wall
    grid[30, 5:35] = 1

    # Add a small block
    grid[5:10, 40:45] = 1

    return grid


if __name__ == "__main__":
    # 1. Build grid
    grid = build_test_grid()

    # 2. Compute safety field
    demo = SafetyFieldDemo(grid, R=6, epsilon=1e-3)
    demo.precomputeSafety()
    preSafety = demo.preSafety

    # 3. Plot obstacle grid and safety field
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    # Plot obstacles
    ax0 = axes[0]
    ax0.set_title("Obstacle Grid")
    im0 = ax0.imshow(grid, origin="lower", cmap="gray_r")
    ax0.set_xlabel("x")
    ax0.set_ylabel("y")

    # Plot preSafety heatmap (mask obstacles for clarity)
    ax1 = axes[1]
    ax1.set_title("Safety Field", fontsize=16)

    # Prepare safety field: hide obstacle cells
    safety_to_plot = preSafety.copy().astype(float)
    safety_to_plot[grid == 1] = np.nan

    # Plot
    im1 = ax1.imshow(safety_to_plot, origin="lower")

  

    # Tick label sizes
    ax1.tick_params(axis='both', which='major', labelsize=14)

    # Colorbar with larger label
    cbar = fig.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04)
    cbar.set_label("Safety Field Value", fontsize=12)
    cbar.ax.tick_params(labelsize=14)


    plt.tight_layout()
    plt.show()
