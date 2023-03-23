#!/bin/bash

# user set:
bindir=/home/szellmann/point-clouds/build
benchdir=/home/szellmann/point-clouds-benchmarks
outdir=${benchdir}/synthetic
orbit_count="--orbit 50"
orbit_center="--orbit-center 0.000796274 6.25849e-07 0"
orbit_up="--orbit-up 0 1 0"
orbit_radius="--orbit-radius 10"

echo "Create if not exists (outdir): "${outdir}
mkdir -p ${outdir}
cd ${outdir}
echo "Running benchmark in: "$PWD
${bindir}/viewer-headless ${orbit_count} ${orbit_center} ${orbit_up} ${orbit_radius} --benchmark
