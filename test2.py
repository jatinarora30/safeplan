from itertools import product

n = 2   # number of loops
k = 10  # range(k) for each loop

for idx in product(range(k), repeat=n):
    # idx is a tuple like (i0, i1, i2, i3)
    print(idx)
