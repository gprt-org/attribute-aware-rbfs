#!/usr/bin/env python3

import sys
import os
import csv
import numpy as np
import matplotlib
from matplotlib import rc
from matplotlib import cm
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

if len(sys.argv) < 2:
    print("Usage: {} <log>".format(sys.argv[0]))
    sys.exit(1)

radii = []
avg_render_times = []
output_file = None

for arg in sys.argv[1:]:
    if arg == "-o":
        continue

    if os.path.splitext(arg)[1] == ".pdf":
        output_file = arg
        continue

    render_times = []
    with open(arg, "r") as f:
        reader = csv.reader(f, delimiter=';')
        for i in range(4):
            next(reader)

        radius = 0.0
        for row in reader:
            radius = float(row[1])
            render_times.append(float(row[2]))

        avg = sum(render_times)/len(render_times)
        #print(radius)
        #print(avg)
        radii.append(radius)
        avg_render_times.append(avg)

regions_color="#0571b0"
samples_color="#ca0020"

zipped = zip(avg_render_times, radii)
sorted_by_perf = sorted(zipped)

fig, ax = plt.subplots(figsize=(6.4, 2.4))
ax.yaxis.set_ticks_position("left")
ax.xaxis.set_ticks_position("bottom")
ax.set_xlabel("Render Time (ms)")
ax.tick_params(axis="x")

#scatter
x, y = zip(*sorted_by_perf)
ax.scatter(x, y, clip_on=False, color=regions_color)
# polyline
npx = np.array(x);
m, b = np.polyfit(npx, y, 1)
ax.plot(npx, npx*m + b, color=regions_color)

plt.xticks(np.arange(min(x), max(x)+1, 10.0))

if output_file and os.path.splitext(output_file)[1] == ".pdf":
    plt.savefig(output_file, bbox_inches="tight")
    print("Plot Saved to {}".format(output_file))
else:
    plt.show()
