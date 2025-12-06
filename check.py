import numpy as np

# -----------------------------
# Parameters
# -----------------------------
EPS = 1

# theoretical upper bound
def theoretical_bound(R, n, eps=EPS):
    bound = 0.0
    for d in range(1, R+1):
        k_d = (2*d + 1)**n - (2*d - 1)**n   # shell cardinality
        bound += k_d / (d + eps)
    return bound


# Generate worst-case obstacle grid for arbitrary dimension n
# A free "center" point surrounded by obstacles in Chebyshev ball radius R
def make_worst_case_grid(R, n):
    size = 2 * R + 3  # ensures padding around R-ball
    shape = tuple([size] * n)

    grid = np.ones(shape, dtype=np.int8)  # all obstacles
    center = tuple([size//2] * n)

    # Make the center free
    grid[center] = 0
    return grid, center


# sum-inverse-distance safety score (general n-D)
def sum_inverse_safety(grid, center, R, eps=EPS):
    n = grid.ndim
    size = grid.shape[0]
    cx = np.array(center)

    total = 0.0

    # iterate through full grid
    it = np.nditer(grid, flags=['multi_index'])
    for val in it:
        idx = np.array(it.multi_index)
        if grid[tuple(idx)] != 1:
            continue  # only obstacles matter

        d_inf = np.max(np.abs(idx - cx))
        if 0 < d_inf <= R:
            total += 1.0 / (d_inf + eps)

    return total


# -----------------------------
# RUN TESTS
# -----------------------------
if __name__ == "__main__":

    EPS = 1

    for n in [2]:        # test dimensions
        for R in [5]:  # test radii

            grid, center = make_worst_case_grid(R, n)
            numerical = sum_inverse_safety(grid, center, R, EPS)
            theory = theoretical_bound(R, n, EPS)

            print(f"\n=== Dimension n={n}, Radius R={R} ===")
            print(f"Theoretical bound:   {theory:.6f}")
            print(f"Numerical sum:       {numerical:.6f}")
            print(f"Difference:          {abs(theory - numerical):.6e}")

            if abs(theory - numerical) < 1e-6:
                print("✔ Verified (matches upper bound)")
            else:
                print("✘ MISMATCH — check logic")
