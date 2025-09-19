# Define the dictionary
sdfCosts = {"12": 2, "aw": 0}

# Sort by value
sorted_sdfCosts = {k: v for k, v in sorted(sdfCosts.items(), key=lambda item: item[1])}

print("Original:", sdfCosts)
print("Sorted by values:", sorted_sdfCosts)

# Access the first key from the original dict
sdfCost = next(iter(sdfCosts))
print("First key in sdfCosts:", sdfCost)

# Access the first key from the sorted dict
sdfCost_sorted = next(iter(sorted_sdfCosts))
print("First key in sorted_sdfCosts:", sdfCost_sorted)

