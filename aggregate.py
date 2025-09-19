#!/usr/bin/env python3
import json
import pathlib
from collections import defaultdict

def mean(nums):
    return sum(nums) / len(nums) if nums else None

def aggregate_results():
    dir_path = pathlib.Path("results")
    output_file = dir_path / "main.json"

    if not dir_path.exists():
        raise SystemExit("Directory 'results' not found")

    accum = defaultdict(lambda: defaultdict(list))

    # collect data
    for file in dir_path.glob("*.json"):
        if file.name == "main.json":
            continue
        try:
            with file.open("r") as f:
                data = json.load(f)
        except Exception as e:
            print(f"Skipping {file.name}: {e}")
            continue

        if not isinstance(data, dict):
            continue

        for algo, metrics in data.items():
            if not isinstance(metrics, dict):
                continue
            for metric, val in metrics.items():
                try:
                    v = float(val)
                except (TypeError, ValueError):
                    continue
                accum[algo][metric].append(v)

    # compute means
    result = {}
    for algo, metric_map in accum.items():
        result[algo] = {}
        for metric, values in metric_map.items():
            m = mean(values)
            if m is not None:
                result[algo][metric] = m

    # save
    with output_file.open("w") as f:
        json.dump(result, f, indent=4, sort_keys=True)

    print(f"Wrote aggregated means to {output_file}")

if __name__ == "__main__":
    aggregate_results()
