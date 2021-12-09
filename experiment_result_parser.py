import re

print("Experiment 1 Results")
data = {}
last_experiment = None
with open("experiment1_results.txt", 'r') as results:
    for line in results:
        if line.startswith("Planning"):
            last_experiment = line
            if last_experiment not in data:
                data[last_experiment] = {}
        elif line.startswith("Scale"):
            scale, exp_num, time = re.findall('\((.*?)\)',line)
            scale = int(scale)
            time = float(time)
            if scale not in data[last_experiment]:
                data[last_experiment][scale] = []
            data[last_experiment][scale].append(time)

for exp, scales in data.items():
    print("For variant ", exp)
    for s in scales:
        print(f"\t Scale {s} average time: {sum(scales[s])/len(scales[s])}")

print("Experiment 2 Results")
data = {}
last_experiment = None
with open("experiment2_results.txt", 'r') as results:
    for line in results:
        if line.startswith("Planning"):
            last_experiment = line
            if last_experiment not in data:
                data[last_experiment] = {}
        elif line.startswith("Obstacles"):
            obs, exp_num, time = re.findall('\((.*?)\)',line)
            obs = int(obs)
            time = float(time)
            if obs not in data[last_experiment]:
                data[last_experiment][obs] = []
            data[last_experiment][obs].append(time)

for exp, obss in data.items():
    print("For variant ", exp)
    for o in obss:
        print(f"\t Obstacles {o} average time: {sum(obss[o])/len(obss[o])}")


print("Experiment 3 Results")
data = {}
last_experiment = None
with open("experiment3_results.txt", 'r') as results:
    for line in results:
        if line.startswith("Planning"):
            last_experiment = line
            if last_experiment not in data:
                data[last_experiment] = {}
        elif line.startswith("Gap"):
            gap, exp_num, time = re.findall('\((.*?)\)',line)
            gap = int(gap)
            time = float(time)
            if gap not in data[last_experiment]:
                data[last_experiment][gap] = []
            data[last_experiment][gap].append(time)

for exp, gaps in data.items():
    print("For variant ", exp)
    for g in gaps:
        print(f"\t Gap {g} average time: {sum(gaps[g])/len(gaps[g])}")
