import numpy as np
import matplotlib.pyplot as plt

# Sample O and S between 0 and 1 for n points
n = 100
O = np.linspace(0, 1, n)
S = np.linspace(0, 1, n)
O_grid, S_grid = np.meshgrid(O, S)

# Define OptiSafe index
B = 1 - np.abs(O_grid - S_grid)  # balance term
R = np.sqrt(O_grid**2 + S_grid**2) / np.sqrt(2)  # strength term
OptiSafe = B * R

# Plot the surface
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(O_grid, S_grid, OptiSafe, cmap='viridis')

ax.set_xlabel("O(P)")
ax.set_ylabel("S(P)")
ax.set_zlabel("OptiSafe Index")
ax.set_title("OptiSafe Index Surface")

fig.colorbar(surf, shrink=0.5, aspect=5)
plt.show()
