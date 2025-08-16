import numpy as np
from safeplan.algos.a_star import A_star
from safeplan.evals.path_cost import PathCost
tests = [
    # (start, goal, grid, description)

    # T1: Empty grid, straight line
    ([0, 0], [0, 4], np.zeros((5, 5), dtype=int), "Empty grid, straight path"),

    # T2: Start == Goal
    ([1, 1], [1, 1], np.zeros((3, 3), dtype=int), "Start equals goal"),

    # T3: No path (solid wall)
    ([1, 2], [3, 2], (lambda g: (g.__setitem__((2, slice(None)), 1), g)[1])(np.zeros((5, 5), dtype=int)),
     "No path due to wall"),

    # T4: Goal cell blocked
    ([0, 0], [2, 2], (lambda g: (g.__setitem__((2, 2), 1), g)[1])(np.zeros((3, 3), dtype=int)),
     "Goal is an obstacle"),

    # T5: Start cell blocked
    ([0, 0], [2, 2], (lambda g: (g.__setitem__((0, 0), 1), g)[1])(np.zeros((3, 3), dtype=int)),
     "Start is an obstacle"),

    # T6: Your original maze-like grid
    ([6, 4], [6, 2], np.array([
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1, 0, 1, 0, 1],
        [0, 0, 1, 0, 1, 0, 0, 0, 0, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
        [1, 0, 1, 1, 1, 1, 0, 1, 0, 0],
        [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 0, 0, 1, 0, 0, 1]
    ], dtype=int), "Given example"),

    # T7: Narrow corridor with a turn
    ([1, 1], [4, 4], (lambda g: (g.__setitem__((slice(1,5), 1), 0),
                                  g.__setitem__((4, slice(1,5)), 0), g)[2])(np.ones((6, 6), dtype=int)),
     "Narrow corridor with turn"),

    # T8: Multiple shortest paths
    ([0, 0], [3, 3], np.zeros((4, 4), dtype=int), "Multiple optimal paths"),

    # T9: Diagonal-only path (succeeds only if diagonals allowed)
    ([0, 0], [1, 1], np.array([[0, 1],
                                [1, 0]], dtype=int), "Diagonal-only path"),

    # T10: Invalid start (outside grid)
    ([-1, 0], [2, 2], np.zeros((3, 3), dtype=int), "Invalid start index"),

    # T11: Invalid goal (outside grid)
    ([0, 0], [3, 3], np.zeros((3, 3), dtype=int), "Invalid goal index"),

    # T12: 3D grid with obstacles (N-D support)
    ([0, 0, 0], [2, 2, 2], (lambda g: (
        g.__setitem__((1, 1, slice(None)), 1),
        g.__setitem__((1, slice(None), 1), 1),
        g.__setitem__((slice(None), 1, 1), 1), g)[3])(np.zeros((3, 3, 3), dtype=int)),
     "3D grid with central cross blocked"),
]

# --- Harness ---
planner = A_star()
for i, (s, t, g, desc) in enumerate(tests, 1):
    print(f"\n=== Test {i}: {desc} ===")
    success, path, info = planner.plan(s, t, g)
    print("Success:", success)
    print("Path   :", path)
    print("Info   :", info)


path1 = [(0, 0), (3, 4)]
pc=PathCost()
val1 = pc.eval(start=path1[0], goal=path1[-1], grid=None, path=path1, cellSize=0.1)
#gr=GenerateGrid("../safeplan/safeplan/configs/generate_grid","env1.json")

print(val1)